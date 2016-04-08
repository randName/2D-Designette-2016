import time
from libdw import sm
from soar.io import io

class RobotSensors(sm.SM):

    startState = ( 0, 0, 0 )
    radtodeg = 57.29578

    def __init__( self, botwidth=0.15, validmax=2.0, validmin=0.18 ):
        self.botwidth = botwidth
        self.corridor = 1.5 - botwidth
        self.validmax = validmax
        self.validmin = validmin

    def getNextValues( self, state, inp ):
        dist = [ min( self.validmax, inp.sonars[i] ) for i in (2,4,0) ]
        em = [ i > self.corridor for i in dist ]

        outv = { 'sonar': tuple(dist), 'em': tuple(em) }

        outv['theta'] = int( self.radtodeg*inp.odometry.theta )
        outv['log'] = { 'temp': inp.temperature, 'ldr': inp.light }
        outv['diag'] = [ min( self.validmax, inp.sonars[i] ) for i in (5,3,1) ]

        lrerr, szerr, noValid = 0, 0, True

        for s in (-1,1):
            if self.validmin < dist[s] < self.validmax:
                noValid = False
                lrerr += s*( self.corridor/2 - dist[s] )
                szerr += dist[s] - self.corridor/2

        outv['szerr'] = szerr
        derr = max( -0.02, min( 0.02, lrerr - state[0] ) )
        outv['lrerr'] = state if noValid else ( lrerr, lrerr + state[1], derr )

        return outv['lrerr'], outv

class RobotMover(sm.SM):

    dz = { 'U': 0, 'L': 1, 'R': -1, 'F': 2 }

    def __init__( self, name, endt, pidk, starting=(), log=lambda x: x ):
        self.name = name
        self.startState = starting
        self.log = log
        self.endt = endt
        self.pidk = pidk

    def getNextValues( self, state, inp ):

        a = io.Action()

        if state == 'H':
            return state, a

        curS, path = state

        try:
            setp = int(curS[1:])
        except ValueError:
            setp = curS[1:]

        lrerr = inp['lrerr']
        dist = inp['sonar']
        em = inp['em']

        if curS[0] == 'A':

            # print '\t'.join( "%.3f" % i for i in dist )

            if not path:
                return 'H', a

            if not path[0]:
                ferr = dist[0] - 0.4
                if abs( ferr ) <= 0.1:
                    if setp:
                        tdiff = setp - time.time()
                    else:
                        tdiff = 4 if path[1] == 'X' else 7
                        curS += str( int(time.time()) + tdiff )

                    if tdiff <= 0:
                        print self.log( path[1:] )
                        inp['log']['location'] = path[1]
                        path = path[2:]
                        if path:
                            self.log( inp['log'] )
                            curS = 'U'
                    return ( curS, path ), a
                curS = 'A'
                a.fvel = min( 0.15, 0.2*ferr )
            else:
                nxtS = path[0][0]
                if ( nxtS == 'F' and any( em[1:] ) ) or em[ -1 if nxtS == 'L' else 1 ]:
                    a.fvel = -0.05
                    print "Junction, going %s" % nxtS
                    return ( path[0][0], (path[0][1:],) + path[1:] ), a

                gotobstacle = setp if setp else 0

                if not em[0] and gotobstacle <= 20:
                    ferr = dist[0] - 0.5
                    if abs( ferr ) <= 0.15:
                        gotobstacle += 1
                        if gotobstacle >= 10:
                            return ( 'O', path ), a
                    a.fvel = min( 0.15, 0.25*ferr )
                else:
                    a.fvel = 0.15

                curS = curS[0] + str( gotobstacle )

            a.rvel = sum( lrerr[i]*self.pidk[i] for i in (0,1,2) )

            if a.fvel < 0.15:
                a.rvel *= (a.fvel/0.15)

            return ( curS, path ), a

        elif curS[0] == 'O':

            if not setp:
                setp = inp['theta']
                sweep = ( setp + 325 ) % 360
                stage = 1
            else:
                stage = setp/1000000
                sweep = (setp%1000000)/1000
                setp = setp%1000

            tdiff = ( 360 + sweep - inp['theta'] ) % 360
            if tdiff > 180:
                tdiff -= 360
            dt = 1 if tdiff > 0 else -1

            if stage < 3:
                print dist[0]
                if 0.7 < dist[0] < 2.0:
                    stage = 3
                elif abs( tdiff ) <= 4:
                    stage = 3 - stage
                    sweep = ( setp + ( 323 if stage-1 else 395 ) ) % 360
                else:
                    a.rvel = dt*max( 0.35, min( 0.8, abs( tdiff )*0.008 ) )
            elif stage == 3:
                ferr = dist[0] - 0.3
                if abs( ferr ) <= 0.1:
                    stage = 4
                a.fvel = min( 0.1, 0.25*ferr )

            elif 4 <= stage <= 30:
                tdiff = ( 360 + setp - inp['theta'] ) % 360
                if tdiff > 180:
                    tdiff -= 360
                dt = 1 if tdiff > 0 else -1
                if abs( tdiff ) <= 5:
                    if stage >= 20:
                        if 0.4 <= dist[-1] < 2.0 or 0.4 <= dist[1] < 2.0:
                            stage +=1
                            if stage >= 30:
                                return ( 'A30', path ), a
                        else:
                            stage = 20
                        print stage, dist[-1], dist[1]
                    elif dist[-1] < 0.5 and dist[1] < 0.5:
                        stage += 1
                        if stage >= 10:
                            stage = 20
                    else:
                        stage = 4
                a.fvel = 0.1
                a.rvel = dt*max( 0.35, min( 0.8, abs( tdiff )*0.0075 ) )


            return ( curS[0] + str( setp + sweep*1000 + stage*1000000 ), path ), a

        z = self.dz[curS[0]]
        insidec = 0

        if not setp:
            setp = ( 360 + inp['theta'] + self.endt[z] ) % 360
        elif setp > 400:
            insidec = setp/1000
            setp = setp%1000

        tdiff = ( 360 + setp - inp['theta'] ) % 360
        if tdiff > 180:
            tdiff -= 360

        if abs( tdiff ) <= 5:
            if not any( em[1:] ):
                if not z or insidec >= 15:
                    if z: print "done"
                    return ( 'A', path ), a
                else:
                    insidec += 1
                print insidec,
            else:
                insidec = 0
        else:
            dt = 1 if tdiff > 0 else -1
            limits = ( 0.1, 0.35 ) if z else ( 0.4, 0.8 )
            a.rvel = dt*max( limits[0], min( limits[1], abs( tdiff )*0.0066 ) )

        a.fvel = 0.1 if z else 0

        return ( curS[0] + str( setp + insidec*1000 ), path ), a

    def done( self, state ):
        return state[0] == 'H'

if __name__ == "__builtin__":
    print "Running robot.py as brain"

    path = ('A', ('FF','X','RR','A','LL','X','RF','B','FL','X','FRF','C','FLF','X','RLF','D','RLF','X'))
    # path = ('F', ('','X','R','A','L','X','F','B','F','X','L','C','R','X'))
    endt = ( -168, 90, 0, -90 )
    pidk = ( 0.23, 0, 29.2 )

    def setup():
        robot.behavior = sm.Cascade( RobotSensors(), RobotMover( 'brainSM', endt, pidk, path ) )

    def brainStart():
        print time.strftime("Started at %H:%M:%S")
        robot.behavior.start()

    def step():
        robot.behavior.step(io.SensorInput()).execute()
        io.done(robot.behavior.isDone())

    def brainStop():
        pass

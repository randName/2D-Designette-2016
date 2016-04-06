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
        outv['log'] = ( inp.temperature, inp.light )

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

    def __init__( self, name, endt, starting=(), logfile=None, cloud=None ):
        self.name = name
        self.startState = starting
        self.logfile = logfile
        self.cloud = cloud
        self.endt = endt

    def getNextValues( self, state, inp ):

        a = io.Action()

        if state == 'H':
            return state, a

        curS, path = state

        try:
            setp = int(curS[1:])
        except ValueError:
            setp = None

        lrerr = inp['lrerr']
        szerr = inp['szerr']
        dist = inp['sonar']
        em = inp['em']

        if curS[0] == 'A':

            # print '\t'.join( "%.3f" % i for i in dist )

            if not path:
                return 'H', a

            ferr = 0

            if not path[0] and not em[0]:
                ferr = dist[0] - 0.4
                if abs( ferr ) <= 0.1:
                    if setp:
                        tdiff = setp - time.time()
                    else:
                        tdiff = 1 if path[1] == 'X' else 8
                        curS += str( int(time.time()) + tdiff )

                    if tdiff <= 0:
                        if len( path ) == 2:
                            self.log( path[1] )
                        else:
                            self.log( path[1], data=inp['log'] )
                            curS = 'U'
                        path = path[2:]
                    return ( curS, path ), a

                a.fvel = 0.19*ferr

            elif path[0] and ( ( path[0][0] == 'F' and any( em[1:] ) ) or em[ -1 if path[0][0] == 'L' else 1 ] ):
                a.fvel = -0.05
                a.rvel = 0
                print "Junction, going %s..." % path[0][0],
                return ( path[0][0], (path[0][1:],) + path[1:] ), a

            else:
                a.fvel = 0.15

            pidk = ( 0.23, 0, 29.1 )
            outp = [ lrerr[i]*pidk[i] for i in (0,1,2) ]
            a.rvel = sum( outp )

            if ferr:
                a.rvel *= ferr

            return ( curS, path ), a

        elif curS[0] == 'O':
            print "obstacle"
            return ( curS, path ), a

        z = self.dz[curS[0]]

        if setp:
            tdiff = setp - inp['theta']
        else:
            fin = ( 360 + inp['theta'] + self.endt[z] ) % 360
            curS += str( fin )
            tdiff = fin - inp['theta']

        tdiff = ( 360 + tdiff ) % 360
        if tdiff > 180:
            tdiff -= 360

        if abs( tdiff ) <= 5:
            if not any( em[1:] ) and abs( szerr ) <= 0.5:
                print "arrived"
                return ( 'A', path ), a
        else:
            dt = 1 if tdiff > 0 else -1
            limits = ( 0.1, 0.35 ) if z else ( 0.4, 0.8 )
            a.rvel = dt*max( limits[0], min( limits[1], abs( tdiff )*0.0066 ) )

        a.fvel = 0.1 if z else 0

        return ( curS, path ), a

    def log( self, location, data=None ):
        logstr = time.strftime("<%H:%M:%S> || <%d-%m-%Y> || ")

        if not data:
            logstr += "Finished, arrived at %s" % location
        elif location != 'X':
            logstr += "Expose Plates at %s" % location
            tmf = time.strftime("%H:%M:%S|%d/%m/%y")
            dt = { 'temp': data[0], 'ldr': data[1], 'time': tmf }
        else:
            logstr += "Collect Plates at X"

        print logstr

        if self.logfile:
            self.logfile.write( logstr )

        if self.cloud:
            self.cloud.put( 'station%s' % location, dt )

    def done( self, state ):
        return state[0] == 'H'

if __name__ == "__builtin__":
    print "Running as brain"

    # path = ('A', ('FF','X','RR','A','LL','X','RF','B','FL','X','FRF','C','FLF','X','RLF','D','RR','H'))
    # path = ('F', ('','X','R','A','L','X','F','B','F','X','L','C','R','X'))
    path = ('A',('L','C','R','X'))
    endt = ( -168, 90, 0, -90 )

    def setup():
        robot.behavior = sm.Cascade( RobotSensors(), RobotMover( 'brainSM', endt, path ) )

    def brainStart():
        robot.behavior.start()

    def step():
        robot.behavior.step(io.SensorInput()).execute()
        io.done(robot.behavior.isDone())

    def brainStop():
        pass

import time
from libdw import sm
from soar.io import io

class RobotSensors(sm.SM):

    startState = ( 0, 0 )
    radtodeg = 57.29578
    validmax = 2.0
    validmin = 0.18
    middlesn = 0.7
    botwidth = 0.15
    corridor = 1.5 - botwidth

    def getNextValues( self, state, inp ):
        bearing = int( self.radtodeg*inp.odometry.theta )
        dist = [ min( self.validmax, inp.sonars[i] ) for i in (2,4,0) ]
        em = [ i > self.corridor for i in dist ]

        lrerr, szerr, noValid = 0, 0, True

        for s in (-1,1):
            if self.validmin < dist[s] < self.validmax:
                noValid = False
                lrerr += s*( self.middlesn - dist[s] )
                szerr += dist[s] - self.corridor/2

        E = state if noValid else ( lrerr, szerr )
        logd = ( inp.temperature, inp.light )

        return E, { 'sonar': dist, 'err': ( E, state ), 'theta': bearing, 'em': em, 'log': logd }

class RobotMover(sm.SM):

    dz = { 'U': 0, 'L': 1, 'R': -1, 'F': 2 }

    def __init__( self, name, starting=() ):
        self.name = name
        self.startState = starting

    def getNextValues( self, state, inp ):

        a = io.Action()

        if state == 'H':
            return state, a

        curS, path = state

        try:
            setp = int(curS[1:])
        except ValueError:
            setp = None

        nerr, perr = inp['err']
        dist = inp['sonar']
        em = inp['em']

        if curS[0] == 'A':

            # print '\t'.join( "%.3f" % i for i in dist )

            if not path:
                return 'H', a

            if not path[0] and not em[0]:
                ferr = dist[0] - 0.6
                if abs( ferr ) <= 0.2:
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

                a.fvel = 0.25*ferr

            elif path[0] and ( ( path[0][0] == 'F' and any( em[1:] ) ) or em[ -1 if path[0][0] == 'L' else 1 ] ):
                a.fvel = -0.25
                return ( path[0][0], (path[0][1:],) + path[1:] ), a

            else:
                a.fvel = 0.15

            # if szerr <= -0.1:
            #    return ( 'O', path ), a

            a.rvel = 30*nerr[0] - 29.97*perr[0]

            return ( curS, path ), a

        elif curS[0] == 'O':
            print "obstacle"
            return ( curS, path ), a

        z = self.dz[curS[0]]

        if setp:
            tdiff = setp - inp['theta']
        else:
            endt = ( -162, 90, 0, -90 )
            fin = ( 360 + inp['theta'] + endt[z] ) % 360
            curS += str( fin )
            tdiff = fin - inp['theta']

        if not any( em[1:] ) and abs( tdiff ) <= 5 and abs( nerr[1] ) <= 0.5:
            return ( 'A', path ), a

        if abs( tdiff ) <= 45:
            a.rvel = tdiff*( 0.007 if z else 0.012 )
        elif z:
            a.rvel = 0.3*z
        else:
            a.rvel = -0.7

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

    def done( self, state ):
        return state[0] == 'H'

if __name__ == "__builtin__":
    print "Running as brain"

    # path = ('A', ('FF','X','RR','A','LL','X','RF','B','FL','X','FRF','C','FLF','X','RLF','D','RR','H'))
    # path = ('F', ('','X','R','A','L','X','F','B','F','X','L','C','R','X'))
    path = ('A',('L','C','R','X'))

    def setup():
        robot.behavior = sm.Cascade( RobotSensors(), RobotMover( 'brainSM', path ) )

    def brainStart():
        robot.behavior.start()

    def step():
        robot.behavior.step(io.SensorInput()).execute()
        io.done(robot.behavior.isDone())

    def brainStop():
        pass

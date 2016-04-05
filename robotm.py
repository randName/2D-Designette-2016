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

        return E, { 'sonar': dist, 'err': ( E, state ), 'theta': bearing, 'em': em }

class RobotMover(sm.SM):

    def __init__( self, name, starting=() ):
        self.name = name
        self.startState = starting

    def getNextValues( self, state, inp ):

        a = io.Action()

        if state == 'H':
            return state, a

        curS, path = state

        try:
            if time.time() - curS >= 0:
                state = ( 'U', path )
            return state, a
        except TypeError:
            pass
        
        if curS == 'J': # Junction
            if not path:
                return 'H', a

            if not path[0]:
                if len(path) == 2:
                    self.log( path[1] )
                    return 'H', a

                self.log( path[1], data=inp )
                curS = int(time.time()) + ( 1 if path[1] == 'X' else 8 )

                return ( curS, path[2:] ), a
            
            curS = path[0][0]
            print "At junction, going %s" % curS
            return ( curS, (path[0][1:],) + path[1:] ), a

        nerr, perr = inp['err']
        dist = inp['sonar']
        em = inp['em']

        if curS == 'A': # Alley

            # print '\t'.join( str(round(i,3)) for i in dist )

            # if szerr <= -0.1:
            #    return ( 'O', path ), a

            # a.fvel = 0.1

            return ( curS, path ), a

        elif curS.startswith('O'):
            print "obstacle"
            return ( curS, path ), a

        return ( curS, path ), a

    def log( self, location, data=None ):
        logstr = time.strftime("<%H:%M:%S> || <%d-%m-%Y> || ")

        if not data:
            logstr += "Finished, arrived at %s" % location
        elif location != 'X':
            logstr += "Expose Plates at %s" % location
            tmf = time.strftime("%H:%M:%S|%d/%m/%y")
            dt = { 'temp': data.temperature, 'ldr': 0, 'time': tmf }
        else:
            logstr += "Collect Plates at X"

        print logstr

    def done( self, state ):
        return state[0] == 'H'

if __name__ == "__builtin__":
    print "Running as brain"

    # path = ('A', ('FF','X','RR','A','LL','X','RF','B','FL','X','FRF','C','FLF','X','RLF','D','RR','H'))
    # path = ('F', ('','X','R','A','L','X','F','B','F','X','L','C','R','X'))

    def setup():
        robot.behavior = sm.Cascade( RobotSensors(), RobotMover( 'brainSM', path ) )

    def brainStart():
        robot.behavior.start()

    def step():
        robot.behavior.step(io.SensorInput()).execute()
        io.done(robot.behavior.isDone())

    def brainStop():
        pass

import time
from libdw import sm
from soar.io import io

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

        dist = [ min( 2.5, inp.sonars[i] ) for i in (2,3,4,0,1) ]
        em = ( False, dist[2] > self.corridor, dist[-2] > self.corridor )

        szerr = dist[-2] + dist[2] - self.corridor
        lrerr = dist[-2] - dist[2]
        if abs( lrerr ) <= 0.05: lrerr = 0

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
    path = ( 'A', ('L','X') )

    def setup():
        robot.behavior = RobotMover( 'brainSM', path )

    def brainStart():
        robot.behavior.start()

    def step():
        robot.behavior.step(io.SensorInput()).execute()
        io.done(robot.behavior.isDone())

    def brainStop():
        pass

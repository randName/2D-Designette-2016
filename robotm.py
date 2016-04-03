import time
from libdw import sm
from soar.io import io

class RobotMover(sm.SM):

    # botwidth = 0.08
    botwidth = 0.145
    corridor = 1.5 - botwidth
    
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

            #if any(em) or dist[0] < 0.8:
            #    return ( 'J', path ), a

            # if szerr <= -0.1:
            #    return ( 'O', path ), a
            if szerr <= 0.01:
                szerr = 0

            # a.fvel = 0.1

            print '\t'.join( str(round(i,3)) for i in dist ),

            if lrerr or szerr:
            #     a.fvel = 0.05
            #     a.rvel = 0.2*lrerr # - 4.0*td*szerr
                
                 print "\t%.3f\t%.3f" % ( lrerr, szerr )
            else:
                 print
            
            return ( curS, path ), a

        elif curS.startswith('O'):
            print "obstacle"
            return ( curS, path ), a

        dir = curS[0]
        stage = curS[1:]

        A = ( 'A', path ), a

        if dir == 'F':
            if not any(em):
                return A

            a.fvel = 0.2

        elif dir == 'U':
            if not stage and any(em):
                stage = 'A'
            elif stage == 'A' and szerr <= 0.01:
                return A

            a.rvel = 0.3

        else:
            if stage == 'C' and szerr <= 0.01:
                return A

            z = 1 if dir == 'L' else -1

            if stage == 'B' and not em[z]:
                return A

            if not stage:
                if dist[0] < self.corridor:
                    stage = 'O'
                elif dist[z] < self.corridor:
                    stage = 'A'
                else:
                    stage = 'E'

            elif stage in 'AO' and em[z]:
                stage = 'C' if stage == 'O' else 'B'

            elif stage.startswith('E'):
                if stage == 'E' and em[z]:
                    stage = 'E1'
                elif stage == 'E1' and not em[z]:
                    stage = 'E2'
                elif stage == 'E2' and em[z]:
                    stage = 'B'

            a.fvel = 0.1
            a.rvel = z*0.18

	print a.fvel, a.rvel

        return ( "%s%s" % ( dir, stage ), path ), a

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

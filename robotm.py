import time
from libdw import sm
from soar.io import io

class RobotMover(sm.SM):

    botwidth = 0.08 # 0.12
    corridor = 1.5 - botwidth
    
    def __init__( self, name, starting='' ):
        self.name = name
        self.startState = starting

    def getNextValues( self, state, inp ):

        def dge( i ):
            if dist[i] == 2.5: return 1.0
            e = 1.44*dist[i*2]/dist[i]
            if 0.99 < e < 1.01: return 1.0
            return e

        a = io.Action()

        if state == 'H':
            return state, a

        if isinstance( state, basestring ):
            curS, path = 'A', state
        else:
            curS, path = state

        try:
            if time.time() - curS >= 0:
                state = ( 'J', path )
            return state, a
        except ( TypeError, IndexError ):
            pass

        dist = [ min( 2.5, inp.sonars[i] ) for i in (2,3,4,0,1) ]
        
        if curS == 'J': # Junction
            if not path:
                print 'Finished'
                return 'H', a

            curS = path[0]
            if curS == 'S':
                print 'Waiting, temp: ', inp.temperature
                curS = int(time.time()) + 8
            else:
                print "At junction, going %s" % curS

            return ( curS, path[1:] ), a

        em = ( False, dist[2] > self.corridor, dist[-2] > self.corridor )

        szerr = dist[-2] + dist[2] - self.corridor
        lrerr = dist[-2] - dist[2]
        if abs( lrerr ) <= 0.08: lrerr = 0

        lde, rde = dge(-1), dge(1)
        if lde > 1.0 or rde < 1.0:
            td = 1
        elif lde < 1.0 or rde > 1.0:
            td = -1
        else:
            td = 0
        
        if curS == 'A': # Alley

            if any(em) or dist[0] < 0.8:
                return ( 'J', path ), a

            if szerr <= -0.01:
                return ( 'O', path ), a
            elif szerr <= 0.005:
                szerr = 0

            a.fvel = 0.5

            # print '\t'.join( str(round(i,3)) for i in dist )

            if lrerr or szerr:
                a.fvel = 0.1
                a.rvel = 0.2*lrerr - 4.0*td*szerr
                
                print "%.3f\t%.3f" % ( lrerr, szerr )
            
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

            a.fvel = 0.5

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
                if dist[0] < 2.0:
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

            a.fvel = 0.25
            a.rvel = z*0.3

        return ( "%s%s" % ( dir, stage ), path ), a

    def done( self, state ):
        return state[0] == 'H'

if __name__ == "__builtin__":

    path = 'FFSUFRFSUFRS'

    def setup():
        robot.behavior = RobotMover( 'brainSM', path )

    def brainStart():
        robot.behavior.start()

    def step():
        robot.behavior.step(io.SensorInput()).execute()
        io.done(robot.behavior.isDone())

    def brainStop():
        pass

    def shutdown():
        pass

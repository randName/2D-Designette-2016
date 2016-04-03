import time
from libdw import sm
from soar.io import io

class RobotMover(sm.SM):

    botwidth = 0.08 # 0.12
    corridor = 1.5 - botwidth
    
    def __init__( self, name, starting='' ):
        self.name = name
        self.startState = starting

    def done( self, state ):
        return state[0] == 'H'

    def getNextValues( self, state, inp ):

        def dge( d ):
            if d == 2.5: return 0
            e = d - 1.02
            if abs( e ) <= 0.01: return 0
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
                return 'H', a

            curS = path[0]
            if curS == 'S':
                print 'Waiting, temp: ', inp.temperature
                curS = int(time.time()) + 8
            else:
                curS += str( sum( 1<<i for i in (0,1,2) if dist[i*2-2] < self.corridor ) )

            return ( curS, path[1:] ), a

        em = ( dist[0] < self.corridor, dist[2] > self.corridor, dist[-2] > self.corridor )

        lrerr = dist[-2] - dist[2]
        if abs( lrerr ) <= 0.08: lrerr = 0

        dgerr = ( 0, dge( dist[1] ), dge( dist[-1] ) )
        
        if curS == 'A': # Alley

            if any(em):
                return ( 'J', path ), a

            szerr = dist[-2] + dist[2] - self.corridor

            if szerr <= -0.01:
                return ( 'O', path ), a

            a.fvel = 0.5

            # print '\t'.join( str(round(i,3)) for i in dist )

            outs = "%.3f\t%.3f\t%.3f\t%.3f" % ( ( szerr, lrerr ) + dgerr[1:] )

            if lrerr:
                print "lr", outs
                kz = 0.4 if szerr <= 0.02 else -0.3
                a.fvel = 0.1
                a.rvel = kz*lrerr

            elif szerr >= 0.01:
                print "sz", outs
                kz = 0
                if dgerr[-1] < 0:
                    kz = -2
                if dgerr[1] < 0:
                    kz = 2
                a.fvel = 0.1
                a.rvel = kz*szerr
            
            return ( curS, path ), a

        elif curS.startswith('O'):
            print "obstacle"
            return ( curS, path ), a

        dir = curS[0]
        jtype = int(curS[1])
        stage = curS[2:]

        A = ( 'A', path ), a

        if dir == 'F':
            if not any(em[1:]):
                return A
            a.fvel = 0.5

        elif dir == 'U':
            a.rvel = 0.3

            if not stage and any(em[1:]):
                stage = 'A'
            elif stage == 'A' and not any(em[1:]) and abs( lrerr ) < 0.1:
                return A

        else:
            if stage == 'C' and not any(em[1:]):
                return A

            z = 1 if dir == 'L' else -1

            if stage == 'B' and not em[z]:
                return A

            a = io.Action(fvel=0.25,rvel=z*0.3)

            if jtype:
                if em[z]: stage = 'C' if jtype&2 else 'B'
            else:
                if not stage and not em[z]:
                    stage = 'A'
                elif stage == 'A' and em[z]:
                    stage = 'B'

        return ( "%s%d%s" % ( dir, jtype, stage ), path ), a

if __name__ == "__builtin__":

    path = 'F'

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

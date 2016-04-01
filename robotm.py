import time
from libdw import sm
from soar.io import io

class RobotMover(sm.SM):

    botwidth = 0.08 # 0.12
    corridor = 1.5 - botwidth
    
    def __init__( self, name, starting='' ):
        self.startState = starting

    def done( self, state ):
        return state[0] == 'H'

    def getNextValues( self, state, inp ):

        a = io.Action()

        if isinstance( state, basestring ):
            if state == 'H': return state, a
            curS, path = 'I', state
        else:
            curS, path, jtype = state
            J = ( ( 'J', path, jtype ), a )

        try:
            if time.time() - state[0] >= 0:
                return J
            else:
                return state, a
        except TypeError:
            pass

        dist = [ min(2,i) for i in inp.sonars[:5] ]

        if curS in 'IJ':
            if not path: return 'H', a

            if curS == 'I':
                pass # Init sensors

            curS = path[0]
            if curS == 'S':
                print 'Waiting, temp: ', inp.temperature
                curS = int(time.time()) + 8

            jtype = sum( 1<<i for i in (0,1,2) if dist[3-i] < 2.0 )
            return ( ( curS, path[1:], jtype ), a )

        # print ( "%2s" % curS )

        em = ( dist[0] > self.corridor, dist[4] > self.corridor )
        err = ( dist[0] - dist[4], dist[1] - dist[3] )

        if curS == 'F':

            szerr = dist[0] + dist[4] - self.corridor

            #if jtype == 5 and any(em):
            #   return J
            #if not ( jtype or any(em) ):
            #   return J
            #if jtype == 4 and ( em[0] or not em[1] ):
            #   return J
            #if jtype == 1 and ( em[1] or not em[0] ):
            #   return J

            if dist[2] <= 0.8:
                return J
            #elif dist[2] < 1.8:
            #   a.fvel = 0.3

            # any( jtype&(1<<i) and dist[3-i] > 1.5 for i in (0,2) )
            # if jtype&4 and abs(sur[0]) > 0.5:
            #   fv = 0.3
            #elif jtype&1 and abs(sur[1]) > 0.5:
            #   fv = 0.3
            if False:
                pass
            else:
                a.fvel = 0.1
                print '\t'.join( str(round(i,3)) for i in dist[:5] ),

                if abs(szerr) >= 0.02 or abs(err[0]) >= 0.08:
                    print "\t%.2f\t%.4f" % ( szerr, err[0] )

                    szk = 4.0 if err[0] >= 0 else -4.0

                    a.fvel = 0.05
                    a.rvel = 0.2 * err[0] * ( 1 - szerr )
                else:
                    print

        elif curS.startswith('R'):
            a = io.Action(fvel=0.25,rvel=-0.3)

            if jtype:
                if em[0]: curS = 'R2' if jtype&2 else 'R1'
            else:
                if curS == 'R' and not em[0]:
                    curS = 'R0'
                elif curS == 'R0' and em[0]:
                    curS = 'R1'

            if curS == 'R1' and not em[0]:
                return J

            if curS == 'R2' and not any(em) and abs(sum(err)) < 0.1:
                return J

        elif curS.startswith('L'):
            a = io.Action(fvel=0.25,rvel=0.3)

            if jtype:
                if em[1]: curS = 'L2' if jtype&2 else 'L1'
            else:
                if curS == 'L' and not em[1]:
                    curS = 'L0'
                elif curS == 'L0' and em[1]:
                    curS = 'L1'

            if curS == 'L1' and not em[1]:
                return J

            if curS == 'L2' and not any(em) and abs(sum(err)) < 0.1:
                return J

        elif curS.startswith('U'):
            a.rvel = 0.3

            if curS == 'U' and any(em):
                curS = 'U0'
            elif curS == 'U0' and not any(em) and abs(sum(err)) < 0.1:
                return J

        else:
            print 'What'

        return ( ( curS, path, jtype ), a )

if __name__ == "__builtin__":

    path = 'F'
    # path = 'FLFFFSU'

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

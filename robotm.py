import time
from libdw import sm
from soar.io import io

class RobotMover(sm.SM):

	corridor = 1.5
	
	def __init__( self, name, starting='' ):
		self.startState = starting
	
	def done( self, state ):
		return state[0] == 'H'

	def getNextValues( self, state, inp ):
		
		a = io.Action()

		if isinstance( state, basestring ):
			curS = 'J'
			path = state
		else:
			curS, path, jtype = state
			J = ( ( 'J', path, jtype ), a )

		# print '\t'.join( str(round(i,3)) for i in dist )

		try:
			if time.time() - state[0] >= 0:
				print 'Done'
				return J
			else:
				return state, io.Action()
		except TypeError:
			pass

		# if curS not in 'HJ': print ( "%2s" % curS )

		dist = [ min(2,i) for i in inp.sonars ]
		em = ( dist[0] > self.corridor, dist[4] > self.corridor )
		err = ( dist[0] - dist[4], dist[1] - dist[3] )

		if curS == 'H':
			pass

		elif curS == 'J':
			jtype = sum( 1<<i for i in (0,1,2) if dist[3-i] < 2.0 )
			if path:
				curS = path[0]
				path = path[1:]
				if curS == 'S':
					print 'Waiting...', inp.temperature
					return ( int(time.time())+8, path, jtype ), io.Action()
			else:
				curS = 'H'

		elif curS == 'F':

			if jtype == 5 and any(em):
				return J
			if not ( jtype or any(em) ):
				return J
			if jtype == 4 and ( em[0] or not em[1] ):
				return J
			if jtype == 1 and ( em[1] or not em[0] ):
				return J

			if dist[2] <= 0.8:
				return J
			elif dist[2] < 1.8:
				a.fvel = 0.3

			szerr = dist[0] + dist[4] + 0.08 - self.corridor

			# any( jtype&(1<<i) and dist[3-i] > 1.5 for i in (0,2) )
			# if jtype&4 and abs(sur[0]) > 0.5:
			#	fv = 0.3
			#elif jtype&1 and abs(sur[1]) > 0.5:
			#	fv = 0.3
			if False:
				pass
			else:
				a.fvel = 0.4
				
				if abs(szerr) >= 0.002 or abs(err[0]) >= 0.05:
					print szerr, err[0]
					
					szk = 4.0 if err[0] >= 0 else -4.0
					
					a.fvel = 0.15
					# a.rvel = 0.3*err[0] - szk*szerr

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

	path = 'FLFRFRFRF'
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
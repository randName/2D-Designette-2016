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
		
		if isinstance( state, basestring ):
			curS = 'J'
			path = state
		else:
			curS, path, jtype = state
			J = ( ('J',path,jtype), io.Action() )

		# print '\t'.join( str(round(i,3)) for i in dist )

		try:
			if time.time() - state[0] >= 0:
				print 'Done'
				return J
			else:
				return state, io.Action()
		except TypeError:
			pass
	
		dist = [ min(2,i) for i in inp.sonars ]
		em = ( dist[0] > self.corridor, dist[4] > self.corridor )

		if curS not in 'HJ': print ( "%2s" % curS )
		
		fv = 0
		rv = 0
		
		err = ( dist[0] - dist[4], dist[1] - dist[3] )
		sur = ( sum(dist[0:2])-1.7, sum(dist[3:5])-1.7 )

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
				fv = 0.3

			# any( jtype&(1<<i) and dist[3-i] > 1.5 for i in (0,2) )
			if jtype & 4 and abs(sur[0]) > 0.5:
				fv = 0.3
			elif jtype & 1 and abs(sur[1]) > 0.5:
				fv = 0.3
			else:
				fv = 0.5
				
				if jtype:
					serr = sum(err) if jtype == 5 else 0
				else:
					serr = err[1]

				if abs(serr) > 0.1:
					rv = 0.3*serr
					fv = 0.15

		elif curS.startswith('R'):
			rv = -0.3
			fv = 0.25
			
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
			rv = 0.3
			fv = 0.25
			
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
			rv = 0.3

			if curS == 'U' and any(em):
				curS = 'U0'
			elif curS == 'U0' and not any(em) and abs(sum(err)) < 0.1:
				return J

		else:
			print 'What'
			
		return ( ( curS, path, jtype ), io.Action(fvel=fv,rvel=rv) )

if __name__ == "__main__":
	mover = RobotMover('brainSM','FL')
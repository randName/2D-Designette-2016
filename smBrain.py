import sys, os
sys.path.append( os.getcwd() )

from libdw import sm
from soar.io import io
from time import strftime
from firebase import FirebaseApplication

from pathsearch import Map
from robot import RobotMover, RobotSensors
from challenge import get_targets, logto, log

level = 1
url = 'http://people.sutd.edu.sg/~oka_kurniawan/10_009/y2015/2d/tests/level2_3.inp'

sensor = RobotSensors()

endt = ( -168, 90, 0, -90 )
pidk = ( 0.23, 0, 29.2 )

logf = 'log.txt'
with open( 'firebase.txt' ) as f:
    fburl, fbtok = f.read().split()

log = logto( cloud=FirebaseApplication( fburl, fbtok ), logfile=logf )( log )

mover = RobotMover( 'brainSM', endt, pidk, log=log )

def getRobotStartState( level, path ):
    initstate = 'F'
    if level:
        initstate = 'A'
        path = 'H' + path
    print "Path:", ' -> '.join( path )
    route = Map( level ).walkpath( path )
    return initstate, route

#####################
##  Brain methods  ##
#####################

def setup():
    with open( logf, 'w' ) as f:
        f.write('')
    robot.behavior = sm.Cascade( sensor, mover )

def brainStart():
    path, trips = get_targets( url, False )
    mover.startState = getRobotStartState( level, path )
    print strftime("Started run at %H:%M:%S")
    robot.behavior.start()

def step():
    inp = io.SensorInput()
    robot.behavior.step(inp).execute()
    io.done(robot.behavior.isDone())

def brainStop():
    pass

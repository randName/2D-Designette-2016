import sys, os
sys.path.append( os.getcwd() )

from soar.io import io
from libdw import gfx
from firebase import FirebaseApplication

from pathsearch import Map
from challenge import get_targets
from robot import RobotMover, RobotSensors

with open( 'firebase.txt' ) as f:
     fburl, fbtok = f.read().split()
fb = FirebaseApplication( fburl, fbtok )

logf = open( 'log.txt', 'w' )

path = Map(0).walkpath('XAXCX')
sensor = RobotSensors()
mover = RobotMover('bdSM',('F',path))

#####################
##  Brain methods  ##
#####################

def setup():
    robot.behavior = sm.Cascade( sensor, mover )

def brainStart():
    robot.behavior.start()

def step():
    inp = io.SensorInput()
    robot.behavior.step(inp).execute()
    io.done(robot.behavior.isDone())

def brainStop():
    logf.close()

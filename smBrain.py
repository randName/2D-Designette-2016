import sys, os
sys.path.append( os.getcwd() )

from soar.io import io
from libdw import gfx
from robotm import RobotMover
from pathsearch import Map

path = Map(0).walkpath('XAXCX')
mover = RobotMover('bdSM',('F',path))

#####################
##  Brain methods  ##
#####################

def setup():
    robot.behavior = mover

def brainStart():
    robot.behavior.start()

def step():
    inp = io.SensorInput()
    robot.behavior.step(inp).execute()
    io.done(robot.behavior.isDone())

def brainStop():
    pass

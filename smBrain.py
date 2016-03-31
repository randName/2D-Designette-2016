import sys, os
sys.path.append( os.getcwd() )

from soar.io import io
from libdw import gfx
from robotm import RobotMover

mover = RobotMover('bdSM','FFF')

#####################
##  Brain methods  ##
#####################

def setup():
	robot.gfx = gfx.RobotGraphics()
	robot.behavior = mover

def brainStart():
	robot.behavior.start(traceTasks=robot.gfx.tasks())

def step():
	inp = io.SensorInput()
	robot.behavior.step(inp).execute()
	io.done(robot.behavior.isDone())

def brainStop():
	pass

def shutdown():
	pass

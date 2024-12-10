from jetbot import Robot
import time

#pip install traitlets
#pip install Adafruit-MotorHAT
#pip install sparkfun-qwiic

def smooth_stop(robot, steps):
	left_value = robot.left_motor.value
	right_value = robot.right_motor.value

	left_stepsize = left_value / steps
	right_stepsize = right_value / steps
	
	while left_value > 0 or right_value > 0:
		left_value -= left_stepsize
		right_value -= right_stepsize

		robot.left_motor.value = left_value
		robot.right_motor.value = right_value

		time.sleep(0.5 / steps)

def smooth_start(robot, steps, speed):
	left_value = robot.left_motor.value
	right_value = robot.right_motor.value

	left_stepsize = (speed - left_value) / steps
	right_stepsize = (speed - right_value) / steps
	
	while left_value < speed or right_value < speed:
		left_value += left_stepsize
		right_value += right_stepsize

		robot.left_motor.value = left_value
		robot.right_motor.value = right_value

		time.sleep(0.5 / steps)
#Movement
robot = Robot()

robot.left(0.3)
time.sleep(0.5)
smooth_stop(robot, 4)

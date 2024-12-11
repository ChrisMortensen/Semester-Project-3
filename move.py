from jetbot import Robot
import time
import threading
from sensorInput import getSensorInput
from sensorInput import zRotation
from sensorInput import distance

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

def forward(angle, speed):
	angleDiff = angle-zRotation
	if abs(angleDiff) < 1.5:
		robot.forward(speed)
	elif angleDiff < 0:
		robot.left(speed * abs(angleDiff) / 180)
	elif angleDiff > 0:
		robot.right(speed * abs(angleDiff) / 180)

def drive_until(dist, speed):
	angle = zRotation
	while distance > dist:
		forward(angle, speed)
		time.sleep(0.2)
	robot.stop()

#Sensorinput 
thread = threading.Thread(target=getSensorInput)
thread.daemon = True
thread.start()

#Robot Actions
robot = Robot()

drive_until(20, 0.1)

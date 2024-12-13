from jetbot import Robot
import time
import threading
from sensorInput import *

#pip install traitlets
#pip install Adafruit-MotorHAT
#pip install sparkfun-qwiic

map = []		# The map of points
offset = [0,0] 	# The position of the robot in the room relative to its original position
offset[0] = 0
offset[1] = 0

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
	print("3")
	angleDiff = angle-zRotation
	if abs(angleDiff) < 1.5:
		print("4")
		robot.forward(speed)
	elif angleDiff < 0:
		print("5")
		robot.left(speed)# * abs(angleDiff) / 180)
	elif angleDiff > 0:
		print("6")
		robot.right(speed)# * abs(angleDiff) / 180)
	print("7")

def drive_until(dist, speed):
	print("1")
	angle = zRotation
	initialDistance = distance
	while distance > dist:
		print("2")
		forward(angle, speed)
		time.sleep(0.4)
	print("8")
	robot.stop()
	dDist = initialDistance - distance
	print("9")
	calcOffset(dDist, angle)

def calcOffset(dist, angle):
	angle_radians = math.radians(angle)
	print("10")
	# Calculate the x and y components of the unit vector, then scale by distance
	x = dist * math.cos(angle_radians)	# x component scaled by distance
	y = dist * math.sin(angle_radians)	# y component scaled by distance
	offset[0] += x
	offset[1] += y
	print("11")

def map():
	x, y = rawPoints[-1]
	map.append(x + offset[0], y + offset[1])

#Sensorinput 
thread = threading.Thread(target=getSensorInput)
thread.daemon = True
thread.start()
time.sleep(2)

#Robot Actions
robot = Robot()
print("0")
drive_until(20, 0.1)
print("12")
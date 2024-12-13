from jetbot import Robot
import time
import threading
from sensorInput import *
import math
import serial

#pip install traitlets
#pip install Adafruit-MotorHAT
#pip install sparkfun-qwiic


# Specify the serial port and baud rate
arduino_port = "/dev/ttyACM0"	# Replace with your Arduino's port
baud_rate = 9600				# Match this with the Arduino's baud rate

# Initial values
zRotation = 0 	# Rotation of the robot
distance = 0	# Distance to the wall from the distancesensor
rawPoints = []	# The values for the points in the room
map = []		# The map of points
offset = [0,0] 	# The position of the robot in the room relative to its original position
offset[0] = 0
offset[1] = 0
turnSpeed = 0.4

def findPointValue(points, zRotation, distance):
	# Convert zRotation to radians
	zRotation_radians = math.radians(zRotation)
	
	# Calculate the x and y components of the unit vector, then scale by distance
	x = distance * math.cos(zRotation_radians)	# x component scaled by distance
	y = distance * math.sin(zRotation_radians)	# y component scaled by distance
	points.append((x,y))

def getSensorInput():
	global zRotation, distance
	while True:
		if ser.in_waiting > 0:
			input_data = ser.readline().decode('utf-8').strip()	# Read line, decode to string, and strip newline
			try:
				gyro_input, distance_input = input_data.split()
				zRotation = float(gyro_input)	# Assuming the gyro returns numeric values
				distance = float(distance_input)
				
				# Find the new x and y values based on the current zRotation and distance
				findPointValue(rawPoints, zRotation, distance)
			except ValueError:
				pass	# Ignore if the value is not a valid float
			
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
	if abs(angleDiff) < 3:
		robot.forward(speed)
	elif angleDiff < 0:
		robot.left(speed * abs(angleDiff) / turnSpeed)
	elif angleDiff > 0:
		robot.right(speed * abs(angleDiff) / turnSpeed)

def drive_until(dist, speed):
	angle = zRotation
	initialDistance = distance
	while distance > dist:
		forward(angle, speed)
		time.sleep(0.2)
	robot.stop()
	dDist = initialDistance - distance
	calcOffset(dDist, angle)

def calcOffset(dist, angle):
	angle_radians = math.radians(angle)
	# Calculate the x and y components of the unit vector, then scale by distance
	x = dist * math.cos(angle_radians)	# x component scaled by distance
	y = dist * math.sin(angle_radians)	# y component scaled by distance
	offset[0] += x
	offset[1] += y

def map():
	x, y = rawPoints[-1]
	map.append(x + offset[0], y + offset[1])

#Sensorinput 
ser = serial.Serial(arduino_port, baud_rate, timeout=1)
time.sleep(2)
thread = threading.Thread(target=getSensorInput)
thread.daemon = True
thread.start()
time.sleep(2)

#Robot Actions
robot = Robot()
print("Stating")
#drive_until(20, 0.1)
robot.left(0.5 * turnSpeed)
print("Done")
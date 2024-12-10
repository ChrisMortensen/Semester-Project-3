import math
import time
import serial

# Specify the serial port and baud rate
arduino_port = "COM4"  # Replace with your Arduino's port
baud_rate = 9600	   # Match this with the Arduino's baud rate

# Initial values
zRotation = 0 	# Rotation of the robot
distance = 0	# Distance to the wall from the distancesensor
xOffset = 0		# The position of the robot in the room relative to its original position
yOffset = 0
xPoints = []	# The values for the points in the room
yPoints = []

ser = serial.Serial(arduino_port, baud_rate, timeout=1)
time.sleep(2)

def findPointValue(values, zRotation, distance):
    # Convert zRotation to radians
    zRotation_radians = math.radians(zRotation)
    
    # Calculate the x and y components of the unit vector, then scale by distance
    values[0] = distance * math.cos(zRotation_radians)  # x component scaled by distance
    values[1] = distance * math.sin(zRotation_radians)  # y component scaled by distance

def getSensorInput():
	global zRotation, distance
	while True:
		if ser.in_waiting > 0:
			input_data = ser.readline().decode('utf-8').strip()  # Read line, decode to string, and strip newline
			gyro_input, distance_input = input_data.split()
			try:
				zRotation = float(gyro_input)  # Assuming the gyro returns numeric values
				distance = float(distance_input)

				values = [0, 0]
				
				# Find the new x and y values based on the current zRotation and distance
				findPointValue(values, zRotation, distance)

				# Append the new point to the lists
				xPoints.append(values[0] + xOffset)
				yPoints.append(values[1] + yOffset)
			except ValueError:
				pass  # Ignore if the value is not a valid float

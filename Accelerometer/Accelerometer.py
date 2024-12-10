import serial
import time
import threading
from collections import deque
import numpy as np

# Specify the serial port and baud rate
arduino_port = "COM4"  # Replace with your Arduino's port
baud_rate = 9600	   # Match this with the Arduino's baud rate
ser = serial.Serial(arduino_port, baud_rate, timeout=1)
time.sleep(2)

lock = threading.Lock()

accX_list = deque(maxlen=100)  
accY_list = deque(maxlen=100)
accZ_list = deque(maxlen=100)

biasX, biasY, biasZ = 0.0, 0.0, 0.0

velocity = [0.0, 0.0, 0.0]  # Initial velocity in m/s
position = [0.0, 0.0, 0.0]  # Initial position in meters

def getAccValues():
	while True:
		if ser.in_waiting > 0:
			input_data = ser.readline().decode('utf-8').strip()
			try:
				x, y, z = map(float, input_data.split())
				with lock:  # Ensure thread-safe access
					accX_list.append(x)
					accY_list.append(y)
					accZ_list.append(z)
			except ValueError:
				print("Invalid data received...")

def calculateAverages():
	with lock:  # Ensure thread-safe access
		if accX_list and accY_list and accZ_list:
			avgX = sum(accX_list) / len(accX_list)
			avgY = sum(accY_list) / len(accY_list)
			avgZ = sum(accZ_list) / len(accZ_list)
			# Clear the lists after computing averages
			accX_list.clear()
			accY_list.clear()
			accZ_list.clear()
			return avgX, avgY, avgZ
		else:
			return None, None, None

def updateVelocityAndPosition(accX, accY, accZ):
	global velocity, position, last_time
	
	current_time = time.time()
	dt = current_time - last_time  # Time difference in seconds
	last_time = current_time
	
	# Update velocity (v = u + at)
	velocity[0] += accX * dt
	velocity[1] += accY * dt
	velocity[2] += accZ * dt

	# Update position (s = s0 + v * t)
	position[0] += velocity[0] * dt
	position[1] += velocity[1] * dt
	position[2] += velocity[2] * dt

	return velocity, position

def calibrate():
	print("Calibrating... Please keep the device steady.")
	calibration_duration = 5  # Duration for calibration in seconds
	start_time = time.time()

	tempX, tempY, tempZ = [], [], []
	
	while time.time() - start_time < calibration_duration:
		avgX, avgY, avgZ = calculateAverages()
		if avgX is not None:
			tempX.append(avgX)
			tempY.append(avgY)
			tempZ.append(avgZ)
		time.sleep(0.1)  # Sampling rate during calibration

	# Calculate biases as the average of the collected values
	global biasX, biasY, biasZ
	biasX = sum(tempX) / len(tempX)
	biasY = sum(tempY) / len(tempY)
	biasZ = sum(tempZ) / len(tempZ)

thread = threading.Thread(target=getAccValues)
thread.daemon = True
thread.start()
last_time = time.time()

calibrate()

try:
	while True:
		avgX, avgY, avgZ = calculateAverages()
		if avgX is not None:
			# Subtract the calibration biases
			avgX -= biasX
			avgY -= biasY
			avgZ -= biasZ	
			velocity, position = updateVelocityAndPosition(avgX, avgY, avgZ)
			print(f"Velocity: {velocity}")
			print(f"Position: {position}")
		time.sleep(0.1)  # Sampling rate
except KeyboardInterrupt:
	print("Loop terminated by user.")
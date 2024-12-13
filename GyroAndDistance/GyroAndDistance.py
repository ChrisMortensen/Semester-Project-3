import matplotlib.pyplot as plt
import serial
import time
import threading
import math

# Initialize empty lists to store points
xpoints = []
ypoints = []

plt.ion()  # Turn on interactive mode for real-time plotting
fig, ax = plt.subplots()

# Specify the serial port and baud rate
arduino_port = "dev/ttyAMC0"  # Replace with your Arduino's port
baud_rate = 9600	   # Match this with the Arduino's baud rate

zRotation = 0  # Initial value for rotation

ser = serial.Serial(arduino_port, baud_rate, timeout=1)
time.sleep(2)

def getGyroValue():
    global zRotation
    while True:
        if ser.in_waiting > 0:
            input_data = ser.readline().decode('utf-8').strip()  # Read line, decode to string, and strip newline
            try:
                gyro_input, distance_input = input_data.split()
                zRotation = float(gyro_input)  # Assuming the gyro returns numeric values
                distance = float(distance_input)
                
                values = [0, 0]
                
                # Find the new x and y values based on the current zRotation and distance
                findPointValue(values, zRotation, distance)

                # Append the new point to the lists
                xpoints.append(values[0])
                ypoints.append(values[1])

            except ValueError:
                pass  # Ignore if the value is not a valid float

# Start the thread for reading gyro values
thread = threading.Thread(target=getGyroValue)
thread.daemon = True
thread.start()

def findPointValue(values, zRotation, distance):
    # Convert zRotation to radians
    zRotation_radians = math.radians(zRotation)
    
    # Calculate the x and y components of the unit vector, then scale by distance
    values[0] = distance * math.cos(zRotation_radians)  # x component scaled by distance
    values[1] = distance * math.sin(zRotation_radians)  # y component scaled by distance

try:
    while plt.fignum_exists(fig.number):  # Check if the window is open
        # Clear the plot and re-plot with updated points
        ax.clear()
        
        # Plot the points
        ax.plot(xpoints, ypoints, 'o')
        
        # Set the limits based on the maximum distance value observed
        max_distance = max([abs(x) for x in xpoints] + [abs(y) for y in ypoints], default=1)
        ax.set_xlim(-max_distance-10, max_distance+10)  # Add some padding
        ax.set_ylim(-max_distance-10, max_distance+10)
        
        # Update the graph
        plt.draw()
        plt.pause(0.1)  # Pause to see the update
except KeyboardInterrupt:
    print("Loop terminated by user.")

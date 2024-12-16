from jetbot import Robot
import time
import threading
import math
import serial

#pip install traitlets
#pip install Adafruit-MotorHAT
#pip install sparkfun-qwiic

robot = Robot()

# Specify the serial port and baud rate
arduino_port = "/dev/ttyACM0"    # Replace with your Arduino's port
baud_rate = 9600                # Match this with the Arduino's baud rate

# Initial values
zRotation = 0     # Rotation of the robot
zRotationRaw = 0
distance = 0    # Distance to the wall from the distancesensor
rawPoints = []    # The values for the points in the room
map = []        # The map of points
offset = [0,0]     # The position of the robot in the room relative to its original position
offset[0] = 0
offset[1] = 0
turnSpeed = 0.001
maxOffsetAngle = 2
firstValues = False
correction_factor = 1.095  # Adjust for hardware-specific calibration
targetAngle = 0


ser = serial.Serial(arduino_port, baud_rate, timeout=1)
time.sleep(2)

def findPointValue(points, zRotation, distance):
    # Convert zRotation to radians
    zRotation_radians = math.radians(zRotation)
    
    # Calculate the x and y components of the unit vector, then scale by distance
    x = distance * math.cos(zRotation_radians)    # x component scaled by distance
    y = distance * math.sin(zRotation_radians)    # y component scaled by distance
    points.append((x,y))

def getSensorInput():
    global zRotation, distance, firstValues, zRotationRaw
    ser.reset_input_buffer()  # Clear the serial input buffer
    while True:
        if ser.in_waiting > 0:
            input_data = ser.readline().decode('utf-8').strip()  # Read line, decode to string, and strip newline
            try:
                gyro_input, distance_input = input_data.split()

                # Convert inputs to floats
                gyro_input = float(gyro_input)
                distance_input = float(distance_input)

                # Handle first values initialization
                if not firstValues:
                    firstValues = True
                    zRotationOffset = gyro_input

                # Adjust gyro input by the offset
                gyro_input -= zRotationOffset
                
                # Get zRotation before normalization
                zRotationRaw = gyro_input

                # Normalize angle to the range -180 to 180 degrees
                gyro_input = (gyro_input + 180) % 360 - 180

                # Update global variables
                distance = distance_input
                zRotation = gyro_input

                #print(f"Raw-Z | {zRotationRaw}")
                #print(f"zRotation: {zRotation} | Distance: {distance}")

                # Placeholder for additional processing
                # findPointValue(rawPoints, zRotation, distance)

            except ValueError as e:
                # Log the error for debugging purposes
                print(f"Error parsing input: {input_data}, error: {e}")
            
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
    global zRotationRaw, maxOffsetAngle, turnSpeed, correction_factor
    angleDiff = angle-zRotationRaw
    if abs(angleDiff) < maxOffsetAngle:
        robot.left_motor.value = speed
        robot.right_motor.value = speed * correction_factor
    else:
        if angleDiff > 0:
            #print("Turn more left | " + str(angleDiff))
            robot.right_motor.value += turnSpeed
        if angleDiff < 0:
            #print("Turn more right | " + str(angleDiff))
            robot.left_motor.value += turnSpeed

def drive_until(dist, speed):
    global distance, targetAngle
    angle = targetAngle
    # Start driving
    robot.left_motor.value = speed
    robot.right_motor.value = speed * correction_factor
    while distance > dist:
        forward(angle, speed)
    robot.stop()
    #dDist = initialDistance - distance
    #calcOffset(dDist, angle)

def calcOffset(dist, angle):
    angle_radians = math.radians(angle)
    # Calculate the x and y components of the unit vector, then scale by distance
    x = dist * math.cos(angle_radians)    # x component scaled by distance
    y = dist * math.sin(angle_radians)    # y component scaled by distance
    offset[0] += x
    offset[1] += y

def map():
    x, y = rawPoints[-1]
    map.append(x + offset[0], y + offset[1])

def rotate_right(degrees):
    global zRotationRaw, correction_factor, targetAngle, robot
    speed = 0.086
    targetAngle = zRotationRaw - degrees

    # Set motor speeds for turning
    robot.left_motor.value = speed
    robot.right_motor.value = -speed * correction_factor
    print(" ")
    while True:
        # Check if the robot has reached or surpassed the target angle
        if math.isclose(zRotationRaw, targetAngle, abs_tol=5):
            break
        time.sleep(0.05)

#Sensorinput 
thread = threading.Thread(target=getSensorInput)
thread.daemon = True
thread.start()
time.sleep(1)

#Robot Actions
print("Stating")
#drive_until(20, 0.2)
targetAngle = zRotationRaw
for x in range(4):
    drive_until(20, 0.2)
    rotate_right(90)

print("Done")
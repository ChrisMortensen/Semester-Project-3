from jetbot import Robot
import time
import threading
import math
import serial
import matplotlib.pyplot as plt
from collections import Counter

robot = Robot()

arduino_port = "/dev/ttyACM0"  
baud_rate = 9600               

# Initial values
zRotation = 0
zRotationRaw = 0
distance = 0    
rawPoints = []    
robot_map = []        
offset = [0,0]     
firstValues = False
correction_factor = 1.095  
targetAngle = 0
normalized_coords = []
lines = []


ser = serial.Serial(arduino_port, baud_rate, timeout=1)
time.sleep(2)

def findPointValue(points, zRotation, distance):
    zRotation_radians = math.radians(zRotation)
    x = distance * math.cos(zRotation_radians)
    y = distance * math.sin(zRotation_radians)
    points.append((x, y))

def getSensorInput():
    global zRotation, distance, firstValues, zRotationRaw, zRotationOffset
    global sensor_thread_running
    sensor_thread_running = True
    ser.reset_input_buffer()
    while sensor_thread_running:
        if ser.in_waiting > 0:
            input_data = ser.readline().decode('utf-8').strip()
            try:
                gyro_input, distance_input = input_data.split()
                gyro_input = float(gyro_input)
                distance_input = float(distance_input)

                if not firstValues:
                    firstValues = True
                    zRotationOffset = gyro_input

                gyro_input -= zRotationOffset
                zRotationRaw = gyro_input
                gyro_input = (gyro_input + 180) % 360 - 180

                distance = distance_input
                zRotation = gyro_input
            except ValueError as e:
                print(f"Error parsing input: {input_data}, error: {e}")

def stop_sensor_thread():
    global sensor_thread_running
    sensor_thread_running = False

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

def render_map_ascii():
    global robot_map, normalized_coords

    # Extracting x and y coordinates from the robot_map
    x_coords = [point[0] for point in robot_map]
    y_coords = [point[1] for point in robot_map]

    # Determine the bounds for the graph (min and max for both axes)
    min_x = min(x_coords) if x_coords else 0
    max_x = max(x_coords) if x_coords else 10
    min_y = min(y_coords) if y_coords else 0
    max_y = max(y_coords) if y_coords else 10

    # Scale the map to fit within the grid size (optional)
    grid_size_x = 20  # Width of the ASCII grid
    grid_size_y = 20  # Height of the ASCII grid

    # Creating an empty grid (list of lists)
    grid = [[' ' for _ in range(grid_size_x)] for _ in range(grid_size_y)]

    # Map the coordinates to the grid
    for x, y in zip(x_coords, y_coords):
        # Normalize x and y coordinates to fit within the grid size
        normalized_x = int((x - min_x) / (max_x - min_x) * (grid_size_x - 1)) if max_x != min_x else grid_size_x // 2
        normalized_y = int((y - min_y) / (max_y - min_y) * (grid_size_y - 1)) if max_y != min_y else grid_size_y // 2
        # Save a copy of normalized x and y so that they can be used outside the function
        normalized_coords.append([(normalized_x, normalized_y)])
        # Place a dot in the grid at the corresponding normalized coordinates
        grid[grid_size_y - 1 - normalized_y][normalized_x] = '.'  # Invert y-axis for printing

    # Printing the grid
    print(f"Map (Grid Size: {grid_size_x}x{grid_size_y}):\n")
    for row in grid:
        print(' '.join(row))

def spin_and_map():
    spin_speed = 0.1
    points_collected = []
    start_rotation = zRotationRaw
    rotations_to_make = 1
    target_rotation = start_rotation + 360 * rotations_to_make
    robot.left_motor.value = -spin_speed
    robot.right_motor.value = spin_speed
    print("Spinning and mapping...")
    
    while True:
        findPointValue(rawPoints, zRotationRaw, distance + 7.5)
        points_collected.append((zRotationRaw, distance))

        if zRotationRaw >= target_rotation:
            break

        time.sleep(0.1)

    robot.stop()
    print("Spinning complete. Mapping points...")
    map_points()

    print("Mapping complete.")

def map_points():
    global robot_map
    for point in rawPoints:
        x, y = point
        mapped_x = x + offset[0]
        mapped_y = y + offset[1]
        robot_map.append((mapped_x, mapped_y))

def connect_points():
    global normalized_coords, lines
    needed_count = 3
    # Extract x and y values

    x_values = [point[0][0] for point in normalized_coords]
    y_values = [point[0][1] for point in normalized_coords]

    

    # Count occurrences using Counter
    x_counts = dict(Counter(x_values))
    y_counts = dict(Counter(y_values))

    # Calculate maxX, maxY, minX, minY for values with counts > 3
    maxX = max([x for x, count in x_counts.items() if count > needed_count], default=None)
    minX = min([x for x, count in x_counts.items() if count > needed_count], default=None)
    maxY = max([y for y, count in y_counts.items() if count > needed_count], default=None)
    minY = min([y for y, count in y_counts.items() if count > needed_count], default=None)

    lines = [
    [(minX, minY), (minX, maxY)],       # Left box wall
    [(minX, maxY), (maxX, maxY)],       # Top box wall
    [(maxX, maxY), (maxX, minY)],       # Right box wall
    [(minX, minY), (maxX, minY)]]       # Bottom box wall
    

def draw_lines():
    global lines

    # Create the plot
    plt.figure(figsize=(8, 6))

    # Draw each line
    for line in lines:
        (x1, y1), (x2, y2) = line
        plt.plot([x1, x2], [y1, y2], 'r-')  # 'r-' means red line

    # Labels and title
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Connected Lines')

    # Display the grid
    plt.grid(True)

    # Show the plot
    plt.show()

def find_robot_position():
    global normalized_coords, robot_position

    # Extract x and y values
    x_values = [point[0][0] for point in normalized_coords]
    y_values = [point[0][1] for point in normalized_coords]

    # Calculate the average position
    avg_x = sum(x_values) / len(x_values) if x_values else 0
    avg_y = sum(y_values) / len(y_values) if y_values else 0

    # Normalize the position to fit within the 20x20 grid
    grid_size_x = 20
    grid_size_y = 20
    min_x = min(x_values) if x_values else 0
    max_x = max(x_values) if x_values else 10
    min_y = min(y_values) if y_values else 0
    max_y = max(y_values) if y_values else 10

    normalized_x = int((avg_x - min_x) / (max_x - min_x) * (grid_size_x - 1)) if max_x != min_x else grid_size_x // 2
    normalized_y = int((avg_y - min_y) / (max_y - min_y) * (grid_size_y - 1)) if max_y != min_y else grid_size_y // 2

    robot_position = (normalized_x, normalized_y)
    print(f"Robot Position (Normalized): {robot_position}")

    return robot_position


# Sensor input thread
thread = threading.Thread(target=getSensorInput)
thread.daemon = True
thread.start()
time.sleep(1)

# Start robot actions
print("Starting spin and mapping...")
spin_and_map()
render_map_ascii()
print(rawPoints)
connect_points()
draw_lines()
print(find_robot_position()) 

stop_sensor_thread()  # Stop sensor input thread after completion
print("Done")

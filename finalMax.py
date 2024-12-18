from jetbot import Robot
import time
import threading
import math
import serial
import matplotlib.pyplot as plt
from collections import Counter

robot = Robot()

# Specify the serial port and baud rate
arduino_port = "/dev/ttyACM0"    # Replace with your Arduino's port
baud_rate = 9600                # Match this with the Arduino's baud rate             

# Initial values
zRotation = 0    # Robot's rotation
zRotationRaw = 0    # Raw value of rotation
distance = 0    # Data from ultrasonic sensor
rawPoints = []    # The values for the points in the room
robot_map = []    # Data for mapping of room
offset = [0,0]    # The position of the robot in the room relative to its original position
offset[0] = 0
offset[1] = 0
turnSpeed = 0.001
maxOffsetAngle = 2        
offset = [0,0]     
firstValues = False
correction_factor = 1.095    # Right wheel is slower than left wheel
targetAngle = 0
normalized_coords = []    # Used for making grid during mapping
lines = []    # Outline for box during mapping
grid = []    # Main grid used in pathfinding
grid_size_x = 20
grid_size_y = 20
maxX = 0


ser = serial.Serial(arduino_port, baud_rate, timeout=1)
time.sleep(2)

def findPointValue(points, zRotation, distance):
    # Convert zRotation to radians
    zRotation_radians = math.radians(zRotation)

    # Calculate the x and y components of the unit vector, then scale by distance
    x = distance * math.cos(zRotation_radians)      # x component scaled by distance
    y = distance * math.sin(zRotation_radians)      # y component scaled by distance
    points.append((x, y))

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

            except ValueError as e:
                # Log the error for debugging purposes
                print(f"Error parsing input: {input_data}, error: {e}")

def smooth_stop(robot, steps):    # Used for stopping less abruptly
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

def smooth_start(robot, steps, speed):    # Used for accelerating slower
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

def forward(angle, speed):    # Drives forward, correcting itself when off-course
    global zRotationRaw, maxOffsetAngle, turnSpeed, correction_factor
    angleDiff = angle-zRotationRaw
    if abs(angleDiff) < maxOffsetAngle:
        robot.left_motor.value = speed
        robot.right_motor.value = speed * correction_factor
    else:     # Correction when off-course
        if angleDiff > 0:
            robot.right_motor.value += turnSpeed
        if angleDiff < 0:
            robot.left_motor.value += turnSpeed

def drive_until(dist, speed):    # Drives until a specified distance to wall
    global distance, targetAngle
    angle = targetAngle
    # Start driving
    robot.left_motor.value = speed
    robot.right_motor.value = speed * correction_factor
    while distance > dist:
        forward(angle, speed)
    robot.stop()

def stop_sensor_thread():
    global sensor_thread_running
    sensor_thread_running = False

def rotate(degrees):
    global zRotationRaw, correction_factor, targetAngle, robot
    speed = 0.086
    targetAngle = zRotationRaw + degrees

    if degrees < 0:
        # Turn right
        robot.left_motor.value = speed
        robot.right_motor.value = -speed * correction_factor
    else:
        # Turn left
        robot.left_motor.value = -speed
        robot.right_motor.value = speed * correction_factor

    while True:
        # Check if the robot has reached or surpassed the target angle
        if math.isclose(zRotationRaw, targetAngle, abs_tol=5):
            break
        time.sleep(0.05)

    robot.stop()

def render_map_ascii():    # Map is drawn in terminal to visualize room
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

def spin_and_map():    # Main function to map the room
    # Robot maps the room by spinning around itself in the middle of the room. Be sure to place robot in the middle.
    spin_speed = 0.1
    start_rotation = zRotationRaw
    rotations_to_make = 1
    target_rotation = start_rotation + 360 * rotations_to_make
    robot.left_motor.value = -spin_speed
    robot.right_motor.value = spin_speed
    print("Spinning and mapping...")
    
    # Spin until all rotations have been made
    while True:
        findPointValue(rawPoints, zRotationRaw, distance + 7.5)
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

def connect_points():    # Connect corners of mapped room
    global normalized_coords, lines, maxX
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

def draw_lines():    # Draw the borders of the mapped room
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

def find_robot_position():    # Establish robot's position in room
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

class Node:    # Each coordinate-point in grid is an object of Node
    def __init__(self, grid, x, y):
        self.x = x
        self.y = y
        self.grid = grid
        self.wall = False
        self.g_score = float('inf')    # Distance from start to current node
        self.f_score = float('inf')    # f_score + g_score (lower = better)

    def get_neighbors(self):
        # Collection of arrays representing the x and y displacement
        rows = len(self.grid)
        cols = len(self.grid[0])
        directions = [[1, 0], [1, 1], [0, 1], [1, -1], [0, -1], [-1, -1], [-1, 0], [-1, 1]]    # Robot can move in 8 directions
        neighbors = []
        for direction in directions:
            neighbor_x = self.x  + direction[0]
            neighbor_y = self.y + direction[1]
            # Add as neighbor if in bounds
            if neighbor_x >= 0 and neighbor_y >= 0 and neighbor_x < cols and neighbor_y < rows:
                neighbors.append(self.grid[neighbor_y][neighbor_x])
        return neighbors

def make_grid():    # Create the main grid
    global grid, grid_size_x, grid_size_y

    def is_on_line(x, y, lines):
        for line in lines:
            # Check for vertical and horizontal line segments
            if (x == line[0][0] and y >= line[0][1] and y <= line[1][1]) or (y == line[0][1] and x >= line[0][0] and x <= line[1][0]):
                return True
        return False

    # Create a node in each cell of grid
    for y in range(grid_size_y):  # Loop over rows (y)
        row_nodes = []
        for x in range(grid_size_x):  # Loop over columns (x)
            node = Node(grid, x, y)  # Create a node at (x, y)
            if is_on_line(x, y, lines):  # Check if the node is part of a wall line
                node.wall = True
            row_nodes.append(node)
        grid.append(row_nodes)

def h_score(start, end):   # Estimated (heuristic) distance from current node to end
    # Calculated using euclidean distance from start node to end note
    return math.sqrt(math.pow((start.x - end.x), 2) + math.pow((start.y - end.y), 2))

def lowest_f_score(node_list):    # Find the node with the lowest (best) f_score in open_list
    final_node = None
    for node in node_list:
        if not final_node or node.f_score < final_node.f_score:
            final_node = node
    return final_node

def reconstruct_path(grid, came_from, current):    # Reconstruct optimal path when pathfinding is done
    path = [current]
    # Uses coordinates as keys in dictionary
    current_key = str(current.x) + ' ' + str(current.y)
    while current_key in came_from:
        current = came_from[current_key]
        current_key = str(current.x) + ' ' + str(current.y)
        path.insert(0, current)
    return path    # Returns optimal path from start node to end node

def a_star(grid, start, end):    # Main pathfinding algorithm
    # Don't let it start or end on a wall
    if start.wall or end.wall:
        print("Start or end node is inside a wall!")
        return
    
    open_set = [start]    # List of nodes to check
    closed_set = []    # List of already-checked or discarded notes
    came_from = {}    # Dictionary used to reconstruct path
    start.g_score = 0
    start.f_score = h_score(start, end)

    # Main loop
    i = 0
    while len(open_set) > 0:
        i += 1
        current = lowest_f_score(open_set)
        open_set.remove(current)
        closed_set.append(current)

        # Return path when end is reached
        if current == end:
            return reconstruct_path(grid, came_from, current)

        # Get list of neighbor-nodes to check
        for neighbor in current.get_neighbors():
            # If node is a wall, skip
            if neighbor in closed_set or neighbor.wall == True:
                continue
            # If both adjacent nodes are walls, dont let it be searched - prevent illegal travel
            adj_node_1 = grid[current.y][neighbor.x]
            adj_node_2 = grid[neighbor.y][current.x]
            if adj_node_1.wall == True and adj_node_2.wall == True:
                continue
            # Add node to open set to check later
            temp_g_score = current.g_score + h_score(current, neighbor)
            if neighbor not in open_set:
                open_set.append(neighbor)
            elif temp_g_score > neighbor.g_score:
                # Not a better path
                continue
            # Found a better path
            came_from[str(neighbor.x) + ' ' + str(neighbor.y)] = current
            neighbor.g_score = temp_g_score
            neighbor.f_score = neighbor.g_score + h_score(neighbor, end)

def followPath(grid, start, end):    # Give instructions to robot
    global zRotationRaw, robot
    completedPath = a_star(grid, start, end)
    currentHeading = 0  # Assuming robot starts facing 'up' (0 degrees)

    for i in range(len(completedPath) - 1):
        # Calculate the movement direction from current to next position
        xOffset = completedPath[i + 1].x - completedPath[i].x
        yOffset = completedPath[i + 1].y - completedPath[i].y
        
        # Calculate the target angle relative to the robot's current heading
        targetHeading = rotateRelativeToPath(xOffset, yOffset, currentHeading)
        
        # Calculate the angle to turn (smallest rotation)
        angleToTurn = targetHeading - currentHeading
        if angleToTurn > 180:
            angleToTurn -= 360  # Turn counter-clockwise (smaller angle)
        elif angleToTurn < -180:
            angleToTurn += 360  # Turn clockwise (smaller angle)

        # Output the move with the turn angle
        print(f"Turning {angleToTurn} degrees to move from position: "
              f"({completedPath[i].x}, {completedPath[i].y}) "
              f"to position ({completedPath[i + 1].x}, {completedPath[i + 1].y}).")

        # Rotate and drive forward
        rotate(angleToTurn)
        currentHeading = targetHeading  # Update robot's heading

        # This is roughly the distance of a node
        forward(zRotationRaw, 0.1)
        time.sleep(0.5)
        robot.stop()
        print(f"Driving forward 5 cm.")
        
        # Update the current heading
        print(f"New heading: {currentHeading} degrees.\n")

def rotateRelativeToPath(x, y, currentHeading):
    # Calculate the target heading based on x and y offset
    # Note: switch-statements not available due to old python version in robot
    if x == 1 and y == 1:
        targetHeading = -45
    elif x == 1 and y == 0:
        targetHeading = -90
    elif x == 1 and y == -1:
        targetHeading = -135
    elif x == 0 and y == 1:
        targetHeading = 0
    elif x == 0 and y == -1:
        targetHeading = -180
    elif x == -1 and y == 1:
        targetHeading = 45
    elif x == -1 and y == 0:
        targetHeading = 90
    elif x == -1 and y == -1:
        targetHeading = 135
    else:
        print("Coords out of range - something is wrong.")
        return None
    
    return targetHeading

#-----------------------------------------------
#   MAIN                                       #
#-----------------------------------------------
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
robotPosition = find_robot_position()
print(robotPosition)
make_grid() 
endNode = grid[3][3]    # Destination coordinates - coordinate (x,y) = grid[y][x]
startNode = grid[robotPosition[1]][robotPosition[0]]
followPath(grid, startNode, endNode)    # followPath runs A*

stop_sensor_thread()  # Stop sensor input thread after completion
print("Done")

import math
minX = 0
minY = 0
maxX = 20
maxY = 20
grid = []
grid_size_x = 20
grid_size_y = 20

lines = [
    [(minX, minY), (minX, maxY)],       # Left box wall
    [(minX, maxY), (maxX, maxY)],       # Top box wall
    [(maxX, maxY), (maxX, minY)],       # Right box wall
    [(minX, minY), (maxX, minY)]]

class Node:
    def __init__(self, grid, x, y):
        self.x = x
        self.y = y
        self.grid = grid
        self.wall = False
        self.g_score = float('inf')
        self.f_score = float('inf')

    def get_neighbors(self):
        # Collection of arrays representing the x and y displacement
        rows = len(self.grid)
        cols = len(self.grid[0])
        directions = [[1, 0], [1, 1], [0, 1], [1, -1], [0, -1], [-1, -1], [-1, 0], [-1, 1]]
        neighbors = []
        for direction in directions:
            neighbor_x = self.x  + direction[0]
            neighbor_y = self.y + direction[1]
            if neighbor_x >= 0 and neighbor_y >= 0 and neighbor_x < cols and neighbor_y < rows:
                neighbors.append(self.grid[neighbor_y][neighbor_x])
        return neighbors

def make_grid():
    global grid, grid_size_x, grid_size_y

    def is_on_line(x, y, lines):
        for line in lines:
            # Check for vertical and horizontal line segments
            if (x == line[0][0] and y >= line[0][1] and y <= line[1][1]) or \
               (y == line[0][1] and x >= line[0][0] and x <= line[1][0]):
                return True
        return False

    for y in range(grid_size_y):  # Loop over rows (y)
        row_nodes = []
        for x in range(grid_size_x):  # Loop over columns (x)
            node = Node(grid, x, y)  # Create a node at (x, y)
            if is_on_line(x, y, lines):  # Check if the node is part of a wall line
                node.wall = True
            row_nodes.append(node)
        grid.append(row_nodes)


def h_score(start, end):
    # Calculated using euclidean distance from start node to end note
    return math.sqrt(math.pow((start.x - end.x), 2) + math.pow((start.y - end.y), 2))

def lowest_f_score(node_list):
    final_node = None
    for node in node_list:
        if not final_node or node.f_score < final_node.f_score:
            final_node = node
    return final_node

def reconstruct_path(grid, came_from, current):
    path = [current]
    current_key = str(current.x) + ' ' + str(current.y)
    while current_key in came_from:
        current = came_from[current_key]
        current_key = str(current.x) + ' ' + str(current.y)
        path.insert(0, current)
    return path

make_grid()
startNode = grid[3][3] # (y, x)
endNote = grid[15][7] # (y, x)

def a_star(grid, start, end):
    open_set = [start]
    closed_set = []
    came_from = {}
    start.g_score = 0
    start.f_score = h_score(start, end)

    i = 0
    while len(open_set) > 0:
        i += 1
        current = lowest_f_score(open_set)
        open_set.remove(current)
        closed_set.append(current)

        if current == end:
            return reconstruct_path(grid, came_from, current)

        for neighbor in current.get_neighbors():
            if neighbor in closed_set or neighbor.wall == True:
                continue
            # If both adjacent nodes are walls, dont let it be searched
            adj_node_1 = grid[current.y][neighbor.x]
            adj_node_2 = grid[neighbor.y][current.x]
            if adj_node_1.wall == True and adj_node_2.wall == True:
                continue
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

path = a_star(grid, startNode, endNote)
for node in path:
    print(node.x, node.y, node.wall, node.g_score ,node.f_score)

def followPath(grid, start, end):
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
        print(f"Turning {angleToTurn:.2f} degrees to move from position: "
              f"({completedPath[i].x}, {completedPath[i].y}) "
              f"to position ({completedPath[i + 1].x}, {completedPath[i + 1].y}).")

        # Rotate and drive forward
        currentHeading = targetHeading  # Update robot's heading
        #driveforward function call here
        print(f"Driving forward 5 cm.")
        
        # Update the current heading
        print(f"New heading: {currentHeading:.2f} degrees.\n")

def rotateRelativeToPath(x, y, currentHeading):
    # Calculate the target heading based on x and y offset
    if x == 1 and y == 1:
        targetHeading = 45
    elif x == 1 and y == 0:
        targetHeading = 90
    elif x == 1 and y == -1:
        targetHeading = 135
    elif x == 0 and y == 1:
        targetHeading = 0
    elif x == 0 and y == -1:
        targetHeading = 180
    elif x == -1 and y == 1:
        targetHeading = -45
    elif x == -1 and y == 0:
        targetHeading = -90
    elif x == -1 and y == -1:
        targetHeading = -135
    else:
        print("Coords out of range - something is wrong.")
        return None
    
    return targetHeading

followPath(grid, startNode, endNote)
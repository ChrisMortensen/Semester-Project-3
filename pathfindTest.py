
minX = 0
minY = 0
maxX = 18
maxY = 18
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

for x in range(grid_size_x):
    row_nodes = []
    for y in range(grid_size_y):
        node = Node(grid, x, y)
        if x == lines[0][0][0] or x == lines[1][1][0] or y == lines[0][0][1] or y == lines[0][1][1]:
            node.wall = True
        row_nodes.append(node)
    grid.append(row_nodes)

print(grid[0][18].wall)
'''
for row in grid:
    for node in row:
        if node.wall:
            print("X", end=" ")
        else:
            print(".", end=" ")
    print()
'''
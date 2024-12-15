import time
import numpy as np
import pylab as pl
from matplotlib import collections as mc
import math

pl.ion()  # Turn on interactive mode for real-time plotting
fig, ax = pl.subplots()

step_size = 2                   # Length in cm pr. move
robot_pos = [20,20]             # Robot position in room
robot_dir = 90                  # Default is right so +90 for up
map_points = []                 # Mapped points
map_lines = []                  # Mapped lines
sens_points = [(),(),(),()]     # Latest sensor points

lines = [                       # Room defined using lines
    [(50, 50), (50, 70)],       # Left box wall
    [(50, 70), (70, 70)],       # Top box wall
    [(70, 70), (70, 50)],       # Right box wall
    [(70, 50), (50, 50)],       # Bottom box wall

    [(0, 0), (0, 100)],         # Left wall
    [(0, 100), (100, 100)],     # Top wall
    [(100, 100), (100, 0)],     # Right wall
    [(100, 0), (0, 0)]]         # Bottom wall

def ray_segment_intersection(ray_origin, ray_dir, lines):
    closest_intersection = None
    closest_line = None
    min_distance = float('inf')

    for line in lines:
        p1, p2 = np.array(line[0]), np.array(line[1])
        ray_origin = np.array(ray_origin)
        ray_dir = np.array(ray_dir)

        v1 = ray_origin - p1
        v2 = p2 - p1
        v3 = np.array([-ray_dir[1], ray_dir[0]])

        dot = np.dot(v2, v3)
        if np.abs(dot) < 1e-6:
            continue  # Lines are parallel

        t1 = np.cross(v2, v1) / dot
        t2 = np.dot(v1, v3) / dot

        if t1 >= 0 and 0 <= t2 <= 1:  # Intersection is along the ray and within the segment
            intersection = ray_origin + t1 * ray_dir
            distance = np.linalg.norm(intersection - ray_origin)
            if distance < min_distance:
                min_distance = distance
                closest_intersection = intersection.tolist()
                closest_line = line

    return closest_line, closest_intersection

def map():
    ray_origin = robot_pos
    for dir in [0, 90, 180, 270]:
        dir = (robot_dir + dir) % 360
        x = math.cos(math.radians(dir))
        y = math.sin(math.radians(dir))
        ray_dir = (x, y)
        line, intersection = ray_segment_intersection(ray_origin, ray_dir, lines)
        if line and intersection is not None:
            intersection[0] = int(intersection[0])
            intersection[1] = int(intersection[1])
            if intersection not in map_points:
                map_points.append(intersection)

#   Not used anymore (was a dynamically updating map)
def update_map_lines(new_point):
    line_found = False
    for line in map_lines:
        for i in [0,1]:
            if abs(line[1][i]-new_point[i]) == step_size:   # If end of line is 1 step away
                if abs(line[0][i]-new_point[i]) > abs(line[0][i]-line[1][i]):  # If line becomes longer with new_point
                    if new_point[1 - i] == line[1][1 - i]:             # If the new point has the same for the non-stepvalue as the last point
                        if new_point[1 - i] == line[0][1 - i]:         # If the new point has the same for the non-stepvalue as the first point
                            line[1] = new_point  # Update lastpoint to current point
                            line_found = True
                            return
    if not line_found:
        if [new_point, new_point] not in map_lines:
            map_lines.append([new_point, new_point])

def make_map():
    connect_points()
    while(merge_lines()):
        continue

def connect_points():
    for point in map_points:
        for other_point in map_points:
            if other_point is not point:
                for i in [0,1]:
                    if point[i] + step_size == other_point[i] and point[1 - i] == other_point[1 - i]:
                        map_lines.append([point,other_point])

def merge_lines():
    for line in map_lines:
        for other_line in map_lines:
            if other_line is not line:
                for i in [0,1]:
                    if line[0][i] == line[1][i] == other_line[0][i] == other_line[1][i]:
                        if other_line[0][1 - i] < line[0][1 - i]:
                            line[0][1 - i] = other_line[0][1 - i]
                        if line[1][1 - i] < other_line[1][1 - i]:
                            line[1][1 - i] = other_line[1][1 - i]
                        map_lines.remove(other_line)
                        return True

def dist_to_wall():
    ray_origin = robot_pos
    dir = robot_dir  # The current direction of the robot
    x = math.cos(math.radians(dir))
    y = math.sin(math.radians(dir))
    ray_dir = (x, y)
    
    # Find the closest wall intersection using the ray_segment_intersection function
    line, intersection = ray_segment_intersection(ray_origin, ray_dir, lines)
    
    if intersection:
        # Calculate the distance between the robot and the intersection point
        return np.linalg.norm(np.array(intersection) - np.array(ray_origin))
    else:
        return float('inf')  # If no intersection, return infinity

def move(dist):
    robot_pos[0] += math.cos(math.radians(robot_dir)) * dist
    robot_pos[1] += math.sin(math.radians(robot_dir)) * dist

def move_until(dist):
        wall_dist = dist_to_wall()
        while wall_dist > dist:
            wall_dist = dist_to_wall()
            move(step_size)
            map()
            make_map()
            update_graph()

def update_graph():
    ax.clear()
    # Create the LineCollection of room_objects and plot it
    lc = mc.LineCollection(lines, color="blue", linewidths=2, alpha=0)
    ax.add_collection(lc)


    # Add the points to the plot
    if map_points:
        x_coords, y_coords = zip(*map_points)  # Separate x and y coordinates
        ax.scatter(x_coords, y_coords, color="red", label="Points", alpha=0)  # Plot points

    # Create the LineCollection of map_lines and plot it
    lc = mc.LineCollection(map_lines, color="purple", linewidths=2, alpha=1)
    ax.add_collection(lc)

    ax.scatter(robot_pos[0], robot_pos[1], color="blue", label="Robot")  # Plot robot

    robot_dir_x = math.cos(math.radians(robot_dir)) * 5
    robot_dir_y = math.sin(math.radians(robot_dir)) * 5
    x = [robot_pos[0], robot_pos[0] + robot_dir_x]
    y = [robot_pos[1], robot_pos[1] + robot_dir_y]

    ax.plot(x, y, color="green", alpha=0.5, linewidth=2)


    # Scale and show the plot
    ax.autoscale()
    ax.margins(0.1)
    ax.legend()  # Optional: Add a legend
    pl.show()
    pl.pause(0.05) # Delay in order to see it as an animation

# Map room
for x in range(4):
    move_until(20)
    robot_dir -= 90 #   Negative is clockwise
make_map()
update_graph()
#line_count = len(map_lines)
#print(f"Number of lines in map_lines: {line_count}")
pl.pause(3) # Enable to watch last frame for longer
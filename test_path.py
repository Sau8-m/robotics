from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction , Axis
from pybricks.robotics import DriveBase
from pybricks.tools import wait

class Environment: # create a grid which maps where the obstacles will be located
    def __init__(self, width, height, obstacles):
        self.width = width
        self.height = height
        self.grid = [[0 for _ in range(width)] for _ in range(height)] # initialize every point in grid to 0
        for (x, y) in obstacles: # obstacles exist
            self.grid[y][x] = 1 # mark obstacle at given coordinate

    def is_free(self, x, y): # check if not an obstacle so long as it is within bounds of environment
        return 0 <= x < self.width and 0 <= y < self.height and self.grid[y][x] == 0

class Robot():
    def __init__(self):
        self.hub = PrimeHub(top_side= Axis.Z, front_side= Axis.X) # set top and front to Z and X, assuming we used IMUs, tried using PID initially
        self.left_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE,reset_angle=True,profile=5) # Port and rotation direction for left motor
        self.right_motor = Motor(Port.D, Direction.CLOCKWISE,reset_angle=True,profile=5) # Port and rotation direction for right motor
        self.drive_base = DriveBase(self.left_motor, self.right_motor, wheel_diameter=56, axle_track=111.5) # Put left and right motor together while defining wheel diameter and axle_track length
        self.drive_base.settings(straight_speed=200, turn_rate=150) # Speed and turn rate of robot
        self.current_angle = 0  # Initial angle, robot facing left (0 degrees)

    def turn_to(self, target_angle):
        turn_angle = target_angle - self.current_angle # Angle to turn to
        
        if turn_angle > 180:
            turn_angle -= 360
        elif turn_angle < -180:
            turn_angle += 360
        
        self.drive_base.turn(turn_angle) # Performs robot turning
        
        self.current_angle = target_angle # Set current angle to new angle turned to (from drive_base.turn(turn_angle))

        
    def move_forward(self, units):
        distance_per_unit = 304  # 304 is 1ft in mm (304.5)
        self.drive_base.straight(units * distance_per_unit) # Move in distance of ft, converted to mm

    def move_to(self, current_point, next_point):
        x1, y1 = current_point
        x2, y2 = next_point

        if x1 == x2:  # Vertical movement
            if y1 < y2:  # Moving up
                self.turn_to(90)
            else:  # Moving down
                self.turn_to(-90)
            self.move_forward(abs(y2 - y1))

        elif y1 == y2:  # Horizontal movement
            if x1 < x2:  # Moving right
                self.turn_to(180)
            else:  # Moving left
                self.turn_to(0)
            self.move_forward(abs(x2 - x1))

    def follow_path(self, path):
        for i in range(len(path) - 1):
            current_point = path[i]
            next_point = path[i + 1]
            self.move_to(current_point, next_point)
            wait(500)  # Pause :)


def astar(env, start, goal):
    open_set = [(0, start)] # Index points traveled
    came_from = {}
    g_score = {start: 0} # Weight on each node, initializing each one for now
    f_score = {start: heuristic(start, goal)} # Distance between the points

    while open_set:
        open_set.sort()
        current = open_set.pop(0)[1]

        if current == goal:
            return reconstruct_path(came_from, current)

        for neighbor in get_neighbors(current, env): # Check list of free nodes near current within env
            tentative_g_score = g_score[current] + 1 # Update g_score

            if tentative_g_score < g_score.get(neighbor, float('inf')): # Check if current g_score < inf --> assume distance to other nodes is inf
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score 
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal) # Total distance from current to neigbors (will be cumulative)
                if neighbor not in [i[1] for i in open_set]:
                    open_set.append((f_score[neighbor], neighbor)) # Adds the new node to the open set

    return None

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) # Returns distance from point a to point b

def get_neighbors(node, env): # Receive current node and grid --> find adjacent nodes that are not obstacles
    neighbors = []
    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]: # assuming changes in grid
        neighbor = (node[0] + dx, node[1] + dy)
        if env.is_free(*neighbor):
            neighbors.append(neighbor)
    return neighbors

def reconstruct_path(came_from, current):
    total_path = [current]
    while current in came_from:
        current = came_from[current]
        total_path.append(current)
    return total_path[::-1] # reverse path to be in the format of start -> end


def main():
    robot = Robot()
	#Enviroment
    width, height = 16, 10
	#Obstacles in coordinates (x,y)
    obstacles = [[3,2], [3, 3], [4, 2], [4, 3], [5, 2], [5, 3], [9, 2], [9, 3], [10, 2], [10, 3], [11, 2], [11, 3], [12, 2], [12, 3], [5, 6], [5, 7], [5, 8], [6, 6], [6, 7], [6, 8], [7, 6], [7, 7], [7, 8]]
	# grid
    env = Environment(width, height, obstacles)

    start = (4, 1)
    goal = (1, 8)
	
	# Get the path from A*
    path = astar(env, start, goal)
	#If the path exist then executed 
    if path:
        print(path)
        robot.follow_path(path)

main()

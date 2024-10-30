from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction , Axis
from pybricks.robotics import DriveBase
from pybricks.tools import wait

class Environment:
    def __init__(self, width, height, obstacles):
        self.width = width
        self.height = height
        self.grid = [[0 for _ in range(width)] for _ in range(height)]
        for (x, y) in obstacles:
            self.grid[y][x] = 1

    def is_free(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height and self.grid[y][x] == 0

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, actual):
        error = setpoint - actual
        self.integral += error
        derivative = error - self.prev_error

        out = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        self.prev_error = error

        return out


class Robot():
    def __init__(self):
        self.hub = PrimeHub(top_side= Axis.Z, front_side= Axis.X)
        self.left_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE,reset_angle=True,profile=5)
        self.right_motor = Motor(Port.D, Direction.CLOCKWISE,reset_angle=True,profile=5)
        self.drive_base = DriveBase(self.left_motor, self.right_motor, wheel_diameter=56, axle_track=111.5)
        self.drive_base.settings(straight_speed=200, turn_rate=150)
        self.current_angle = 0  # Initial angle, facing right (0 degrees)

        self.angle_pid = PID(2.0,1.5,0.8)
        self.distance_pid = PID(1.0,1.0,1.0)
    def turn_to(self, target_angle):
        turn_angle = target_angle - self.current_angle
        
        if turn_angle > 180:
            turn_angle -= 360
        elif turn_angle < -180:
            turn_angle += 360
        
        self.drive_base.turn(turn_angle)
        
        self.current_angle = target_angle

        
    def move_forward(self, units):
        distance_per_unit = 304  # 304 is 1ft in mm (304.5)
        self.drive_base.straight(units * distance_per_unit)

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
    open_set = [(0, start)]
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        open_set.sort()
        current = open_set.pop(0)[1]

        if current == goal:
            return reconstruct_path(came_from, current)

        for neighbor in get_neighbors(current, env):
            tentative_g_score = g_score[current] + 1

            if tentative_g_score < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                if neighbor not in [i[1] for i in open_set]:
                    open_set.append((f_score[neighbor], neighbor))

    return None

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def get_neighbors(node, env):
    neighbors = []
    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        neighbor = (node[0] + dx, node[1] + dy)
        if env.is_free(*neighbor):
            neighbors.append(neighbor)
    return neighbors

def reconstruct_path(came_from, current):
    total_path = [current]
    while current in came_from:
        current = came_from[current]
        total_path.append(current)
    return total_path[::-1]


def main():
    robot = Robot()
	#Enviroment
    width, height = 8, 10
	#Obstacles in coordinates (x,y)
    obstacles = [(1,1), (1, 2), (4, 4), (3,4), (5,5)]
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
# robot_params.py
# Centralized configuration for robot parameters
import math
import numpy as np

# Robot physical dimensions
wheel_radius = 0.115   # m (5 inches) TODO: update
wheel_base = 0.538     # m TODO: update
y_axis_max = 450       # mm TODO: update

# Motor specifications
rated_speed = 24.0855 # rad/s (~230 RPM)
max_linear_speed = 0.3        # m/s (~3 km/h)
max_angular_speed = 0.3  # rad

# Encoder specifications
ticks_per_revolution = 3456  # Encoder ticks per wheel revolution (64*(51+3)) ??

# Motor
min_duty_cycle = 9
max_motor_linear_speed = 2*math.pi*wheel_radius*230/60 - 0.5 # 0.5 safety factor
min_motor_linear_speed = 2*math.pi*wheel_radius*3/100*230/60
motor_comp_factor = 1.05

# Feature constraints
y_axis_alignment_tolerance = 0.005 # m

# Saves data to csv files
log = False

# Homography points
pixel_points = np.array([
    [1291, 953],
    [694, 964],
    [1410, 652],
    [986, 659],
    [563, 668],
    [1385, 242],
    [977, 251],
    [571, 258],
    [982, 451],
    [991, 876],
    [1198, 655],
    [774, 664],
    [1262, 371],
    [699, 382],
], dtype=np.float32)

x = 0
y = 30
ground_points = np.array([
    [x+30, y+30],
    [x+30, y+160],
    [x+95, y+0],
    [x+95, y+95],
    [x+95, y+190],
    [x+190, y+0],
    [x+190, y+95],
    [x+190, y+190],
    [x+142.5, y+95],
    [x+47.5, y+95],
    [x+95, y+47.5],
    [x+95, y+142.5],
    [x+160, y+30],
    [x+160, y+160],
], dtype=np.float32)
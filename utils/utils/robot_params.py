# robot_params.py
# Centralized configuration for robot parameters
import math
import numpy as np

# Robot physical dimensions
wheel_radius = 0.115   # m (5 inches) TODO: update
wheel_base = 0.538     # m TODO: update
y_axis_max = 245       # mm 

# Motor specifications
rated_speed = 24.0855 # rad/s (~230 RPM)
max_linear_speed = 0.3        # m/s (~3 km/h)
max_angular_speed = 0.3  # rad
max_zero_angular_speed = 3.0


# Encoder specifications
ticks_per_revolution = 3456  # Encoder ticks per wheel revolution (64*(51+3)) ??

# Motor
min_duty_cycle = 9
max_motor_linear_speed = 2*math.pi*wheel_radius*230/60 - 0.5 # 0.5 safety factor
min_motor_linear_speed = 2*math.pi*wheel_radius*3/100*230/60
motor_comp_factor = 1.05

# Feature constraints
pid_linear_error_tolerance = 0.005 # m
y_axis_alignment_tolerance = 0.01 # m

# Saves data to csv files
log = False

# Homography points
pixel_points = np.array([
    [1417, 1066],
    [1004, 1072],
    [586, 1079],
    [1284, 934],
    [720, 943], 
    [1002, 861],
    [1403, 650],
    [1202, 652],
    [1000, 654],
    [795, 656],
    [591, 659],
    [996, 452],
    [1268, 377],
    [720, 380],
    [1389, 252],
    [993, 253],
    [591, 256],
], dtype=np.float32)

x = 0
y = 55
ground_points = np.array([
    [x-30, y-30],
    [x-30, y+65],
    [x-30, y+160],
    [x, y],
    [x, y+130],
    [x+17.5, y+65],
    [x+65, y-30],
    [x+65, y+17.5],
    [x+65, y+65],
    [x+65, y+112.5],
    [x+65, y+160],
    [x+112.5, y+65],
    [x+130, y],
    [x+130, y+130],
    [x+160, y-30],
    [x+160, y+65],
    [x+160, y+160],
], dtype=np.float32)

# Autonomous mode
explore_linear_speed = 0.3
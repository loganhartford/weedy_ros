# robot_params.py
# Centralized configuration for robot parameters

import math
import numpy as np

from enum import Enum, auto

class FilterType(Enum):
    ODOMETRY = auto()
    ODOMETRY_IMU = auto()
    CUSTOM_EKF = auto()
    ROS_EKF = auto()


POSITION = 1.0
DOCK = 2.0
UNDOCK = 3.0
WORK = 4.0
TRAVEL = 5.0
ROTATE = 6.0
DONE = 7.0

filter_type = FilterType.ODOMETRY_IMU

USE_MAG = True
POST_RAW = False

# --------------------------
# Robot Physical Dimensions
# --------------------------


wheel_radius = 0.228 / 2    # m
wheel_base = 0.497      # distance between center of wheels
l_a = wheel_base / 2        # distance from center of robot to wheel
l_b = 0.09 # Distance from wheel axis to drill - TODO: update this from CAD
y_axis_max = 219.62        # mm
caster_offset = 0.485

# --------------------------
# Motor Specifications
# --------------------------
rpm = 23                               # Rated speed
rated_speed = rpm * 2 * math.pi / 60.0  # rad/s
max_linear_speed = (2 * math.pi * wheel_radius) * rpm / 60 # m/s
max_angular_speed = 2*max_linear_speed # rad/s
ol_speed_gain = 0.05
cl_speed_gain = 0.05
path_max_linear_speed = 0.15
path_max_angular_speed = path_max_linear_speed * 2


# --------------------------
# Encoder Specifications
# --------------------------
ticks_per_revolution = 64 * 547.5 # 64 ticks per revolution, 515 gear ratio

# --------------------------
# Motor Control Parameters
# --------------------------
min_duty_cycle = 5

# --------------------------
# Feature Constraints
# --------------------------
pid_linear_pos_error_tolerance = 0.001  # m
pid_linear_path_error_tolerance = 0.1  # m
angular_error_tolerance = 0.087       # rad, 5 degree
y_axis_alignment_tolerance = 0.01   # m

# --------------------------
# Logging Configuration
# --------------------------
log = False  # Set to True to save data to CSV files

# --------------------------
# Homography Calibration Points
# --------------------------
pixel_points = np.array([
    [1401, 1053], 
    [929, 1055], 
    [451, 1058], 
    [1302, 949], 
    [555, 951], 
    [1253, 897], 
    [604, 900], 
    [929, 804], 
    [1388, 559], 
    [1160, 558], 
    [928, 558], 
    [696, 558], 
    [463, 559], 
    [927, 317], 
    [1241, 229], 
    [611, 227], 
    [1379, 84], 
    [927, 82], 
    [469, 79]], dtype=np.float32)

x = 0
y = 23
ground_points = np.array([
    [x-24.47, y-23.3],
    [x-24.47, y+86.2],
    [x-24.47, y+195.7],
    [x, y],
    [x, y+172.39],
    [x+11.23, y+10.69],
    [x+11.23, y+161.71],
    [x+33.03, y+86.2],
    [x+90.53, y-23.3],
    [x+90.53, y+31.45],
    [x+90.53, y+86.2],
    [x+90.53, y+140.95],
    [x+90.53, y+195.7],
    [x+148.03, y+86.2],
    [x+169.83, y+10.69],
    [x+169.83, y+161.71],
    [x+205.53, y-23.3],
    [x+205.53, y+86.2],
    [x+205.53, y+195.7],
], dtype=np.float32)

# --------------------------
# Autonomous Mode Settings
# --------------------------
explore_linear_speed = 0.3  # m/s

# --------------------------
# UART Stuff
# --------------------------
weed_removal_byte = 0x87
ack_byte = 0x43
callback_byte = 0x03
ticks_byte = 0xAE
battery_byte = 0x11
left_byte = 0x7E
right_byte = 0x3F
up_byte = 0x5A
down_byte = 0x9A
drill_byte = 0x0F
stop_byte = 0x07
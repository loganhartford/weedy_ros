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

filter_type = FilterType.ODOMETRY_IMU

# --------------------------
# Robot Physical Dimensions
# --------------------------


wheel_radius = 0.228 / 2    # m
wheel_base = 0.497      # distance between center of wheels
l_a = wheel_base / 2        # distance from center of robot to wheel
l_b = 0.09 # Distance from wheel axis to drill - TODO: update this from CAD
y_axis_max = 245.0        # mm
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
path_max_linear_speed = 0.075
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
pid_linear_path_error_tolerance = 0.05  # m
angular_error_tolerance = 0.087       # rad, 5 degree
y_axis_alignment_tolerance = 0.01   # m

# --------------------------
# Logging Configuration
# --------------------------
log = True  # Set to True to save data to CSV files

# --------------------------
# Homography Calibration Points
# --------------------------
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

# Ground calibration points for homography
# (x, y) offsets for the ground coordinate system
x = 0
y = 55
ground_points = np.array([
    [x - 30,    y - 30],
    [x - 30,    y + 65],
    [x - 30,    y + 160],
    [x,         y],
    [x,         y + 130],
    [x + 17.5,  y + 65],
    [x + 65,    y - 30],
    [x + 65,    y + 17.5],
    [x + 65,    y + 65],
    [x + 65,    y + 112.5],
    [x + 65,    y + 160],
    [x + 112.5, y + 65],
    [x + 130,   y],
    [x + 130,   y + 130],
    [x + 160,   y - 30],
    [x + 160,   y + 65],
    [x + 160,   y + 160],
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
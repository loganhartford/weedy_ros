# robot_params.py
# Centralized configuration for robot parameters
import math

# Robot physical dimensions
wheel_radius = 0.103   # m (5 inches) TODO: update
wheel_base = 0.536     # m TODO: update
y_axis_max = 450       # mm TODO: update

# Motor specifications
rated_speed = 24.0855 # rad/s (~230 RPM)
max_linear_speed = 0.3        # m/s (~3 km/h)
max_angular_speed = 0.3  # rad

# Encoder specifications
ticks_per_revolution = 3264  # Encoder ticks per wheel revolution

# Motor
min_duty_cycle = 9
max_motor_linear_speed = 2*math.pi*wheel_radius*230/60 - 0.5 # 0.5 safety factor
min_motor_linear_speed = 2*math.pi*wheel_radius*3/100*230/60

# Feature constraints
y_axis_alignment_tolerance = 0.005 # m

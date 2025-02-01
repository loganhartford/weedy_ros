# robot_params.py
# Centralized configuration for robot parameters
import math

# Robot physical dimensions
wheel_radius = 0.127   # meters (5 inches)
wheel_base = 0.381     # meters (distance between wheels)
y_axis_max = 450       # mm TODO: update

# Motor specifications
rated_speed = 24.0855 # rad/s (~230 RPM)
max_linear_speed = 0.8        # m/s (~3 km/h)
# max_linear_speed = 1.2        # m/s (~3 km/h)
max_angular_speed = 2.0       # rad

# Encoder specifications
ticks_per_revolution = 3264  # Encoder ticks per wheel revolution

# Motor
min_duty_cycle = 4
max_motor_linear_speed = 2*math.pi*wheel_radius*230/60 - 0.5 # 0.5 safety factor
min_motor_linear_speed = 2*math.pi*wheel_radius*3/100*230/60

from nav_msgs.msg import Odometry
from rclpy.clock import Clock

import math
import numpy as np

from utils.robot_params import wheel_radius, wheel_base, ticks_per_revolution
from utils.utilities import create_quaternion_from_yaw

from utils.uart import UART

class Localization:
    def __init__(self):
        # Robot parameters
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.ticks_per_revolution = ticks_per_revolution

        # Odometry state
        self.x = 0.0  # m
        self.y = 0.0  # m
        self.theta = 0.0  # rad
        self.last_ticks_left = None
        self.last_ticks_right = None

        self.clock = Clock()
        self.last_time = self.clock.now()

        self.uart = UART()

    def update_odometry(self):
        try:
            ticks_left, ticks_right, stamp = self.uart.get_ticks()
        except Exception as e:
            # self.get_logger().error(f"Error reading ticks: {e}")
            return None
        
        # Handle tick rollover
        max_ticks = 2**15 - 1
        min_ticks = -2**15

        delta_ticks_left = ticks_left - self.last_ticks_left
        delta_ticks_right = ticks_right - self.last_ticks_right

        if delta_ticks_left > max_ticks / 2:
            delta_ticks_left -= (max_ticks - min_ticks + 1)
        elif delta_ticks_left < min_ticks / 2:
            delta_ticks_left += (max_ticks - min_ticks + 1)

        if delta_ticks_right > max_ticks / 2:
            delta_ticks_right -= (max_ticks - min_ticks + 1)
        elif delta_ticks_right < min_ticks / 2:
            delta_ticks_right += (max_ticks - min_ticks + 1)

        self.last_ticks_left = ticks_left
        self.last_ticks_right = ticks_right
        
        delta_time = (stamp - self.last_time).nanoseconds * 1e-9
        self.last_time = stamp

        # Compute wheel displacements
        d_left = (delta_ticks_left / self.ticks_per_revolution) * (2 * math.pi * self.wheel_radius)
        d_right = (delta_ticks_right / self.ticks_per_revolution) * (2 * math.pi * self.wheel_radius)

        # Compute linear and angular displacements
        d = (d_left + d_right) / 2.0
        delta_theta = (d_right - d_left) / self.wheel_base

        # Midpoint method for pose update
        self.x += d * math.cos(self.theta + delta_theta / 2.0)
        self.y += d * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta

        # Normalize theta to [-pi, pi]
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        # Compute velocities
        linear_velocity = d / delta_time
        angular_velocity = delta_theta / delta_time

        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Pose
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        # odom_msg.pose.pose.orientation = create_quaternion_from_yaw(self.theta)
        odom_msg.pose.pose.orientation.z = self.theta

        # Twist
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity

        
        return odom_msg
    
    def reset_odometry(self):
        self.x = 0.0  # m
        self.y = 0.0  # m
        self.theta = 0.0  # rad

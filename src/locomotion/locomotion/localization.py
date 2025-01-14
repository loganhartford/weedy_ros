import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray

import math
import numpy as np

from robot_params import wheel_radius, wheel_base, ticks_per_revolution
from utilities import create_quaternion_from_yaw

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('Localization')

        self.tick_subscriber = self.create_subscription(Int32MultiArray, '/ticks', self.update_odometry, 1)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 1)

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
        self.history_size = 7
        self.linear_velocity_history = np.zeros(self.history_size)
        self.angular_velocity_history = np.zeros(self.history_size)
        self.history_index = 0

        self.last_time = self.get_clock().now()

    def update_odometry(self, msg):
        if self.last_ticks_left == None and self.last_ticks_right == None:
            self.last_ticks_left, self.last_ticks_right = msg.data
            return
        ticks_left, ticks_right = msg.data
        delta_ticks_left = ticks_left - self.last_ticks_left
        delta_ticks_right = ticks_right - self.last_ticks_right
        self.last_ticks_left = ticks_left
        self.last_ticks_right = ticks_right

        # Compute wheel displacements
        d_left = (delta_ticks_left / self.ticks_per_revolution) * (2 * math.pi * self.wheel_radius)
        d_right = (delta_ticks_right / self.ticks_per_revolution) * (2 * math.pi * self.wheel_radius)

        # Compute linear and angular displacements
        d = (d_left + d_right) / 2.0
        delta_theta = (d_right - d_left) / self.wheel_base

        # Update the pose
        current_time = self.get_clock().now()
        delta_time = (current_time - self.last_time).nanoseconds * 1e-9  # Convert to seconds
        self.last_time = current_time

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
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Pose
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation = create_quaternion_from_yaw(self.theta)

        # Twist
        # Moving average filter
        self.linear_velocity_history[self.history_index] = linear_velocity
        self.angular_velocity_history[self.history_index] = angular_velocity
        self.history_index = (self.history_index + 1) % self.history_size
        avg_linear_velocity = np.mean(self.linear_velocity_history)
        avg_angular_velocity = np.mean(self.angular_velocity_history)

        odom_msg.twist.twist.linear.x = avg_linear_velocity
        odom_msg.twist.twist.angular.z = avg_angular_velocity

        # print(f"X: {self.x}, Y: {self.y}, Theta: {self.theta}")
        print(f"Linear Velocity: {avg_linear_velocity}")

        self.odom_publisher.publish(odom_msg)
    
    def destroy_node(self):
        super().destroy_node()

    


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()

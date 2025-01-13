import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int32MultiArray

from robot_params import wheel_radius, wheel_base, ticks_per_revolution
import math


class LocalizationNode(Node):
    def __init__(self):
        super().__init__('Localization')

        self.tick_subscriber = self.create_subscription(Int32MultiArray, '/ticks', self.update_odometry, 1)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        # Robot parameters
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.ticks_per_revolution = ticks_per_revolution

        # Odometry state
        self.x = 0.0  # m
        self.y = 0.0  # m
        self.theta = 0.0  # rad
        self.last_ticks_left = 0
        self.last_ticks_right = 0

        self.last_time = self.get_clock().now()

    def update_odometry(self, msg):
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
        odom_msg.pose.pose.orientation = self.create_quaternion_from_yaw(self.theta)

        # Twist
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity

        # print(f"X: {self.x}, Y: {self.y}, Theta: {self.theta}")
        # print(f"Linear Velocity: {linear_velocity}, Angular Velocity: {angular_velocity}")

        self.odom_publisher.publish(odom_msg)

    @staticmethod
    def create_quaternion_from_yaw(yaw):
        """
        Create a Quaternion from a yaw angle (in radians).
        """
        return Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0)
        )


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

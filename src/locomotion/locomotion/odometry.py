import math
import numpy as np

from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.time import Time
import rclpy

from utils.utilities import normalize_angle
import utils.robot_params as rp
from utils.utilities import create_quaternion_from_yaw, create_yaw_from_quaternion


class OdometryNode(Node):
    """
    Newt Kinematics Model
    https://www.ri.cmu.edu/pub_files/pub3/muir_patrick_1986_1/muir_patrick_1986_1.pdf
    """
    def __init__(self):
        super().__init__('odometry_node')

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.ticks_sub = self.create_subscription(Int32MultiArray, '/ticks', self.ticks_callback, 1)

        self.last_ticks_left = None
        self.last_ticks_right = None

        self.odom = Odometry()
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_link"
        self.odom.pose.pose.position.x = 0.0
        self.odom.pose.pose.position.y = 0.0
        self.odom.pose.pose.position.z = 0.0
        self.odom.twist.twist.linear.x = 0.0
        self.odom.twist.twist.linear.y = 0.0
        
        self.odom.pose.pose.orientation = create_quaternion_from_yaw(0.0)
        self.odom.twist.twist.angular.z = 0.0

        self.max_ticks = 2**15 - 1
        self.min_ticks = -2**15
        
        self.clock = Clock()
        self.last_time= self.clock.now()

        self.get_logger().info("Odometry Initialized")
    

    def ticks_callback(self, msg):
        ticks_left, ticks_right = msg.data[0], msg.data[1]
        
        stamp = self.clock.now()
        delta_time = (stamp - self.last_time).nanoseconds * 1e-9
        self.last_time = stamp
        
        # On the first run, load values
        if self.last_ticks_left is None or self.last_ticks_right is None:
            self.last_ticks_left = ticks_left
            self.last_ticks_right = ticks_right
            return None
        
        delta_left = ticks_left - self.last_ticks_left
        delta_right = ticks_right - self.last_ticks_right

        # Adjust for rollover of int 16 register
        delta_left = self.adjust_ticks(delta_left)
        delta_right = self.adjust_ticks(delta_right)

        self.last_ticks_left = ticks_left
        self.last_ticks_right = ticks_right

        # One encoder counts down
        effective_delta_left = -delta_left
        effective_delta_right = delta_right

        # Wheel angular displacements
        theta_left = (effective_delta_left / rp.ticks_per_revolution) * (2 * math.pi)
        theta_right = (effective_delta_right / rp.ticks_per_revolution) * (2 * math.pi)
        thetas = np.array([[theta_left], 
                           [theta_right]])

        transform = np.array([[rp.l_a,      rp.l_a],
                              [rp.l_b,     -rp.l_b],
                              [-1,          1]])
        scaling = rp.wheel_radius / (2*rp.l_a)
        displacements = np.matmul(transform, thetas) * scaling

        delta_x_robot = displacements[0][0]
        delta_y_robot = displacements[1][0]
        delta_theta = displacements[2][0]

        # Convert to global frame
        old_yaw = create_yaw_from_quaternion(self.odom.pose.pose.orientation)
        delta_x = delta_x_robot * math.cos(old_yaw) - delta_y_robot * math.sin(old_yaw)
        delta_y = delta_x_robot * math.sin(old_yaw) + delta_y_robot * math.cos(old_yaw)
        
        vel_x = delta_x / delta_time
        vel_y = delta_y / delta_time
        w = delta_theta / delta_time

        new_yaw = old_yaw + delta_theta
        new_yaw = normalize_angle(new_yaw)

        self.odom.header.stamp = stamp.to_msg()
        self.odom.pose.pose.position.x += delta_x
        self.odom.pose.pose.position.y += delta_y
        self.odom.twist.twist.linear.x = vel_x
        self.odom.twist.twist.linear.y = vel_y
        self.odom.pose.pose.orientation = create_quaternion_from_yaw(new_yaw)
        self.odom.twist.twist.angular.z = w

        self.odom_pub.publish(self.odom)

    def log_odom(self):
        pose = self.odom.pose.pose
        pos = pose.position
        orient = pose.orientation
        timestamp = Time.from_msg(self.odom.header.stamp).nanoseconds / 1e9
        with open(self.odom_log, "a") as file:
            file.write(f"{timestamp},{pos.x},{pos.y},{pos.z},{orient.z},{orient.w}\n")

    def adjust_ticks(self, delta):
        if delta > self.max_ticks / 2:
            return delta - (self.max_ticks - self.min_ticks + 1)
        elif delta < self.min_ticks / 2:
            return delta + (self.max_ticks - self.min_ticks + 1)
        return delta


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()

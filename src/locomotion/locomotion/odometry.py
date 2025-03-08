import math

from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.time import Time
import rclpy

from utils.utilities import normalize_angle
import utils.robot_params as rp


class OdometryNode(Node):
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
        self.odom.pose.pose.orientation.z = 0.0
        self.odom.twist.twist.linear.x = 0.0
        self.odom.twist.twist.angular.z = 0.0

        self.max_ticks = 2**15 - 1
        self.min_ticks = -2**15
        
        self.clock = Clock()
        self.last_time= self.clock.now()

        if rp.log:
            self.ticks_log = "/mnt/shared/weedy_ros/src/locomotion/locomotion/outputs/ticks_log.csv"
            self.odom_log="/mnt/shared/weedy_ros/src/locomotion/locomotion/outputs/odom_log.csv"
            with open(self.ticks_log, "w") as file:
                file.write("ticks_left,ticks_right\n")
            with open(self.odom_log, "w") as file:
                file.write("Timestamp,X,Y,Z,Orientation_Z,Orientation_W\n")

        self.get_logger().info("Odometry Initialized")
    

    def ticks_callback(self, msg):
        ticks_left, ticks_right = msg.data[0], msg.data[1]
        
        # On the first run, load values
        if self.last_ticks_left is None or self.last_ticks_right is None:
            self.last_ticks_left = ticks_left
            self.last_ticks_right = ticks_right
            return None
        
        delta_left = ticks_left - self.last_ticks_left
        delta_right = ticks_right - self.last_ticks_right

        # Adjust for rollover
        delta_left = self.adjust_ticks(delta_left)
        delta_right = self.adjust_ticks(delta_right)

        self.last_ticks_left = ticks_left
        self.last_ticks_right = ticks_right

        if rp.log:
            with open(self.ticks_log, "a") as file:
                file.write(f"{ticks_left},{ticks_right}\n")

        stamp = self.clock.now()
        delta_time = (stamp - self.last_time).nanoseconds * 1e-9
        self.last_time= stamp

        # Adjust tick sign (assumes left encoder counts down when moving forward)
        effective_delta_left = -delta_left
        effective_delta_right = delta_right

        # Convert ticks to wheel displacements
        d_left = (effective_delta_left / rp.ticks_per_revolution) * (2 * math.pi * rp.wheel_radius)
        d_right = (effective_delta_right / rp.ticks_per_revolution) * (2 * math.pi * rp.wheel_radius)

        # Compute average displacement and heading change
        d = (d_left + d_right) / 2.0
        delta_theta = (d_right - d_left) / rp.wheel_base

        # Compute velocities
        linear_velocity = d / delta_time
        angular_velocity = delta_theta / delta_time

        self.odom.header.stamp = stamp.to_msg()
        self.odom.pose.pose.position.x += d * math.cos(self.odom.pose.pose.orientation.z + delta_theta / 2.0)
        self.odom.pose.pose.position.y += d * math.sin(self.odom.pose.pose.orientation.z + delta_theta / 2.0)
        self.odom.pose.pose.orientation.z += delta_theta
        self.odom.pose.pose.orientation.z = normalize_angle(self.odom.pose.pose.orientation.z)
        self.odom.twist.twist.linear.x = linear_velocity
        self.odom.twist.twist.angular.z = angular_velocity

        if rp.log:
            self.log_odom()

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

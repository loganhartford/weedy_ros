import math
import time

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile
import rclpy
import message_filters

from utils.robot_params import wheel_radius, wheel_base, ticks_per_revolution, log
from utils.utilities import normalize_angle

KALMAN_FILTER = False

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')

        self.pose_pub = self.create_publisher(PoseStamped, '/pose', 10)

        if KALMAN_FILTER:
            qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)
            self.imu_sub=message_filters.Subscriber(self, Imu, "/imu", qos_profile=qos)
            self.ticks_sub=message_filters.Subscriber(self, Int32MultiArray, "/ticks", qos_profile=qos)
            time_syncher=message_filters.ApproximateTimeSynchronizer([self.ticks_sub, self.imu_sub], queue_size=10, slop=0.1)
            time_syncher.registerCallback(self.fusion_callback)
        else:
            self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 1)
            self.ticks_sub = self.create_subscription(Int32MultiArray, '/ticks', self.ticks_callback, 1)
        
        self.last_imu = None
        self.last_ticks_left = None
        self.last_ticks_right = None

        # Robot parameters
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.ticks_per_revolution = ticks_per_revolution

        self.pose = PoseStamped()
        self.pose.header.frame_id = "odom"
        self.pose.pose.position.x = 0.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.orientation.z = 0.0

        self.odom = Odometry()
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_link"
        self.odom.pose.pose.position.x = 0.0
        self.odom.pose.pose.position.y = 0.0
        self.odom.pose.pose.orientation.z = 0.0
        self.odom.twist.twist.linear.x = 0.0
        self.odom.twist.twist.angular.z = 0.0

        # Clock for timestamping
        self.clock = Clock()
        self.last_time = self.clock.now()

        # Setup logging if enabled
        if log:
            self.ticks_log = "/mnt/shared/weedy_ros/src/locomotion/locomotion/outputs/ticks_log.csv"
            self.pose_log="/mnt/shared/weedy_ros/src/locomotion/locomotion/outputs/pose_log.csv"
            with open(self.ticks_log, "w") as file:
                file.write("ticks_left,ticks_right\n")
            with open(self.pose_log, "w") as file:
                file.write("Timestamp,X,Y,Z,Orientation_Z,Orientation_W\n")

        self.get_logger().info("Localization Initialized")
    
    def fusion_callback(self, ticks_msg: Int32MultiArray, imu_msg: Imu):
        self.compute_odometry(ticks_msg.data[0], ticks_msg.data[1], ticks_msg.data[2])

    def imu_callback(self, msg: Imu):
        self.last_imu = msg

    def ticks_callback(self, msg):
        self.compute_odometry(msg.data[0], msg.data[1])

        self.pose.header.stamp = self.odom.header.stamp
        self.pose.pose = self.odom.pose.pose
        self.pose_pub.publish(self.pose)

    def compute_odometry(self, ticks_left, ticks_right):
        # On the first run, load values
        if self.last_ticks_left is None or self.last_ticks_right is None:
            self.last_ticks_left = ticks_left
            self.last_ticks_right = ticks_right
            return None

        # Define tick rollover limits (INT16)
        max_ticks = 2**15 - 1
        min_ticks = -2**15

        delta_left = ticks_left - self.last_ticks_left
        delta_right = ticks_right - self.last_ticks_right

        # Adjust for rollover
        delta_left = self.adjust_ticks(delta_left, max_ticks, min_ticks)
        delta_right = self.adjust_ticks(delta_right, max_ticks, min_ticks)

        self.last_ticks_left = ticks_left
        self.last_ticks_right = ticks_right

        if log:
            with open(self.ticks_log, "a") as file:
                file.write(f"{ticks_left},{ticks_right}\n")

        stamp = self.clock.now()
        delta_time = (stamp - self.last_time).nanoseconds * 1e-9
        self.last_time = stamp

        # Adjust tick sign (assumes left encoder counts down when moving forward)
        effective_delta_left = -delta_left
        effective_delta_right = delta_right

        # Convert ticks to wheel displacements
        d_left = (effective_delta_left / self.ticks_per_revolution) * (2 * math.pi * self.wheel_radius)
        d_right = (effective_delta_right / self.ticks_per_revolution) * (2 * math.pi * self.wheel_radius)

        # Compute average displacement and heading change
        d = (d_left + d_right) / 2.0
        delta_theta = (d_right - d_left) / self.wheel_base

        # Compute velocities
        linear_velocity = d / delta_time
        angular_velocity = delta_theta / delta_time

        self.odom.header.stamp = stamp.to_msg()
        self.odom.pose.pose.position.x += d * math.cos(self.pose.pose.orientation.z + delta_theta / 2.0)
        self.odom.pose.pose.position.y += d * math.sin(self.pose.pose.orientation.z + delta_theta / 2.0)
        self.odom.pose.pose.orientation.z += delta_theta
        self.odom.pose.pose.orientation.z = normalize_angle(self.odom.pose.pose.orientation.z)
        self.odom.twist.twist.linear.x = linear_velocity
        self.odom.twist.twist.angular.z = angular_velocity

        if log:
            self.log_odom()

    def log_odom(self):
        pose = self.odom.pose.pose
        pos = pose.position
        orient = pose.orientation
        timestamp = Time.from_msg(self.odom.header.stamp).nanoseconds / 1e9
        with open(self.pose_log, "a") as file:
            file.write(f"{timestamp},{pos.x},{pos.y},{pos.z},{orient.z},{orient.w}\n")

    def adjust_ticks(self, delta, max_ticks, min_ticks):
        if delta > max_ticks / 2:
            return delta - (max_ticks - min_ticks + 1)
        elif delta < min_ticks / 2:
            return delta + (max_ticks - min_ticks + 1)
        return delta


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

import math
import numpy as np

from std_msgs.msg import Int32MultiArray, Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
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
        self.reset_odom_sub = self.create_subscription(Bool, '/reset_odom', self.reset_odom_callback, 1)

        if rp.filter_type == rp.FilterType.ODOMETRY_IMU:
            self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 1)
        
        self.last_ticks_left = None
        self.last_ticks_right = None
        self.max_ticks = 2**16

        self.last_imu = None

        self.clock = Clock()
        self.last_time= self.clock.now()

        self.imu = None

        self.odom = Odometry()
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_link"
        self.reset_odom_callback(None)

        self.total_ticks = [0, 0]
        log_file="/mnt/shared/weedy_ros/src/locomotion/locomotion/outputs/total_ticks.csv"
        if rp.log:
            self.log_file = log_file
            with open(self.log_file, "w") as file:
                file.write("TicksLeft,TicksRight\n")

        self.get_logger().info("Odometry Initialized")
    
    def imu_callback(self, msg):
        self.imu = msg

    def ticks_callback(self, msg):
        ticks_left, ticks_right = msg.data[0], msg.data[1]
        
        stamp = self.clock.now()
        delta_time = (stamp - self.last_time).nanoseconds * 1e-9
        self.last_time = stamp

        if self.imu is None and rp.filter_type == rp.FilterType.ODOMETRY_IMU:
            return
        
        # On the first run, load values
        if self.last_ticks_left is None or self.last_ticks_right is None:
            self.last_ticks_left = ticks_left
            self.last_ticks_right = ticks_right
            return
        
        delta_left_ticks = ticks_left - self.last_ticks_left
        delta_right_ticks = ticks_right - self.last_ticks_right

        self.last_ticks_left = ticks_left
        self.last_ticks_right = ticks_right

        delta_left_ticks = -float(self.adjust_ticks(delta_left_ticks)) # counts down going forward
        delta_right_ticks = float(self.adjust_ticks(delta_right_ticks))

        self.total_ticks[0] += delta_left_ticks
        self.total_ticks[1] += delta_right_ticks
        if rp.log:
            with open(self.log_file, "a") as file:
                file.write(f"{self.total_ticks[0]},{self.total_ticks[1]}\n")

        # Wheel angular displacements
        w_left = (delta_left_ticks / rp.ticks_per_revolution) * (2 * math.pi) / delta_time
        w_right = (delta_right_ticks / rp.ticks_per_revolution) * (2 * math.pi) / delta_time
        wheel_velocities = np.array([[w_left], 
                           [w_right]])

        transform = np.array([[rp.l_a,      rp.l_a],
                              [0.0,         0.0],
                              [-1.0,        1.0]])
        scaling = rp.wheel_radius / (2*rp.l_a)
        displacements = scaling * (transform @ wheel_velocities)

        vel_x_robot = displacements[0][0]
        w = displacements[2][0]

        old_yaw = create_yaw_from_quaternion(self.odom.pose.pose.orientation)
        vel_x = vel_x_robot * math.cos(old_yaw)
        vel_y = vel_x_robot * math.sin(old_yaw)
        
        delta_x = vel_x * delta_time
        delta_y = vel_y * delta_time
        delta_theta = w * delta_time

        new_yaw = old_yaw + delta_theta
        new_yaw = normalize_angle(new_yaw)

        # imu hack
        if rp.filter_type == rp.FilterType.ODOMETRY_IMU:
            new_yaw = create_yaw_from_quaternion(self.imu.orientation)
            delta_theta = new_yaw - old_yaw
            w = delta_theta / delta_time

        self.odom.header.stamp = stamp.to_msg()
        self.odom.pose.pose.position.x += delta_x
        self.odom.pose.pose.position.y += delta_y
        self.odom.twist.twist.linear.x = vel_x
        self.odom.twist.twist.linear.y = vel_y
        self.odom.pose.pose.orientation = create_quaternion_from_yaw(new_yaw)
        self.odom.twist.twist.angular.z = w

        self.odom_pub.publish(self.odom)
    
    def reset_odom_callback(self, msg):
        self.odom.header.stamp = self.clock.now().to_msg()
        self.odom.pose.pose.position.x = 0.0
        self.odom.pose.pose.position.y = 0.0
        self.odom.pose.pose.position.z = 0.0
        self.odom.pose.pose.orientation = create_quaternion_from_yaw(0.0)
        self.odom.twist.twist.linear.x = 0.0
        self.odom.twist.twist.linear.y = 0.0
        self.odom.twist.twist.angular.z = 0.0

        self.odom_pub.publish(self.odom)

    def log_odom(self):
        pose = self.odom.pose.pose
        pos = pose.position
        orient = pose.orientation
        timestamp = Time.from_msg(self.odom.header.stamp).nanoseconds / 1e9
        with open(self.odom_log, "a") as file:
            file.write(f"{timestamp},{pos.x},{pos.y},{pos.z},{orient.z},{orient.w}\n")

    def adjust_ticks(self, delta):
        if delta > self.max_ticks / 2: # Counting down case
            return delta - self.max_ticks
        elif delta < -(self.max_ticks / 2): # Counting up case
            return delta + self.max_ticks
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

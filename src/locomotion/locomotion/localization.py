import math
import numpy as np

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

from utils.utilities import create_yaw_from_quaternion
import utils.robot_params as rp
from locomotion.kalman_filter import KalmanFilter


class LocalizationNode(Node):

    def __init__(self):
        super().__init__('localization_node')

        self.pose_pub = self.create_publisher(PoseStamped, '/pose', 10)

        if rp.filter_type == rp.FilterType.CUSTOM_EKF:
            qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)
            self.imu_sub=message_filters.Subscriber(self, Imu, "/imu", qos_profile=qos)
            self.ticks_sub=message_filters.Subscriber(self, Odometry, "/odom", qos_profile=qos)
            time_syncher=message_filters.ApproximateTimeSynchronizer([self.ticks_sub, self.imu_sub], queue_size=10, slop=0.1)
            time_syncher.registerCallback(self.fusion_callback)
        elif rp.filter_type == rp.FilterType.ODOMETRY or rp.filter_type == rp.FilterType.ODOMETRY_IMU:
            self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        elif rp.filter_type == rp.FilterType.ROS_EKF:
            self.ekf_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 1)
        
        self.last_imu = None

        self.pose = PoseStamped()
        self.pose.header.frame_id = "odom"
        self.pose.pose.position.x = 0.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.orientation.z = 0.0

        self.kf = None

        self.clock = Clock()
        self.last_time = self.clock.now()

        self.get_logger().info("Localization Initialized")

    def odom_callback(self, msg):
        self.pose.header.stamp = msg.header.stamp
        self.pose.pose = msg.pose.pose
        self.pose.pose.orientation.z = create_yaw_from_quaternion(self.pose.pose.orientation)
        self.pose_pub.publish(self.pose)

    def fusion_callback(self, odom_msg: Odometry, imu_msg: Imu):
        if self.kf == None:
            x = np.array([  odom_msg.pose.pose.position.x,
                            odom_msg.pose.pose.position.y,
                            odom_msg.pose.pose.orientation.z,
                            0,
                            0,
                            0])        

            Q=1.0*np.eye(6)
            R=0.1*np.eye(4)
            P=Q.copy()
            
            self.kf=KalmanFilter(P, Q, R, x)
        
        stamp = self.clock.now()
        delta_time = (stamp - self.last_time).nanoseconds * 1e-9
        self.last_time= stamp

        z=np.array([odom_msg.twist.twist.linear.x,
                    odom_msg.twist.twist.angular.z,
                    imu_msg.linear_acceleration.x,
                    imu_msg.linear_acceleration.y])
        self.kf.predict(delta_time)
        self.kf.update(z)
        
        xhat=self.kf.get_states()
        
        self.pose.pose.position.x = xhat[0]
        self.pose.pose.position.y = xhat[1]
        self.pose.pose.orientation.z = xhat[2]
        self.pose.header.stamp = stamp.to_msg()
        self.pose_pub.publish(self.pose)


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

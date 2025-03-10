#!/mnt/shared/weedy_ros/src/locomotion/locomotion/pwm_venv/bin/python3
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Float32MultiArray

from locomotion.motor_control import MotorController
from locomotion.pid import PID_ctrl
from utils.utilities import calculate_linear_error, calculate_angular_error, float32_multi_array_to_two_d_array, calculate_positioning_error, calculate_rotation_error
import utils.robot_params as rp


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller')

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(PoseStamped, '/pose', self.pose_callback, 1)
        self.create_subscription(Float32MultiArray, '/path', self.path_callback, 10)
        self.create_subscription(Float32MultiArray, '/rotate', self.rotate_callback, 10)
        self.create_subscription(Float32MultiArray, '/position', self.position_callback, 10)
        self.cmd_publisher = self.create_publisher(String, '/cmd', 10)

        self.vel_req = Twist()
        self.pose = None
        self.path = []
        self.rotate = []
        self.new_position = []

        self.motor_controller = MotorController()

        self.positioning_linear_pid = PID_ctrl(
            kp=1.0, kd=0.0, ki=0.0,
            log_file="/mnt/shared/weedy_ros/src/locomotion/locomotion/outputs/lin_pid_log.csv"
        )
        self.positioning_angular_pid = PID_ctrl(
            kp=5.0, kd=0.0, ki=0.0,
            log_file="/mnt/shared/weedy_ros/src/locomotion/locomotion/outputs/ang_pid_log.csv"
        )

        self.path_linear_pid = PID_ctrl(
            kp=1.0, kd=0.0, ki=0.0,
            log_file="/mnt/shared/weedy_ros/src/locomotion/locomotion/outputs/path_lin_pid_log.csv"
        )
        self.path_angular_pid = PID_ctrl(
            kp=1.0, kd=0.0, ki=0.0,
            log_file="/mnt/shared/weedy_ros/src/locomotion/locomotion/outputs/path_ang_pid_log.csv"
        )

        self.control_timer = self.create_timer(0.01, self.control_loop)

        self.get_logger().info("Controller Initialized")
       
    def control_loop(self):
        if self.pose is not None and self.new_position != []:
            # self.get_logger().info("closed_loop_positioning")
            self.closed_loop_positioning()
        elif self.pose is not None and self.path != []:
            # self.get_logger().info("closed_loop_path_following")
            self.close_loop_path_following()
        elif self.pose is not None and self.rotate != []:
            # self.get_logger().info("closed_loop_rotate")
            self.close_loop_rotatation()
        else:
            self.open_loop_control()

    def closed_loop_positioning(self):
        linear_error = calculate_positioning_error(self.pose.pose, self.new_position[-1])
        angular_error = calculate_angular_error(self.pose.pose, self.new_position[-1])

        if abs(linear_error) < rp.pid_linear_pos_error_tolerance:
            self.reset_control()
            self.get_logger().info("new_pos_reached")
            self.cmd_publisher.publish(String(data="new_pos_reached"))
            return

        linear_vel = self.positioning_linear_pid.update([linear_error, self.pose.header.stamp])
        angular_vel = self.positioning_angular_pid.update([angular_error, self.pose.header.stamp])

        self.motor_controller.set_velocity(linear_vel, angular_vel)

    def close_loop_path_following(self):
        # Compute linear error to final goal and angular error to next goal.
        angular_goal = self.look_far_for(self.pose.pose, self.path)
        linear_error = calculate_linear_error(self.pose.pose, self.path[-1])
        angular_error = calculate_angular_error(self.pose.pose, angular_goal)
        
        if linear_error < rp.pid_linear_path_error_tolerance:
            # NOTE: may only want to clear history if we are at the end of the path
            self.reset_control()
            self.get_logger().info("path_complete")
            self.cmd_publisher.publish(String(data="path_complete"))
            return
    
        linear_vel = self.path_linear_pid.update([linear_error, self.pose.header.stamp])
        angular_vel = self.path_angular_pid.update([angular_error, self.pose.header.stamp])
        
        self.motor_controller.set_velocity(linear_vel, angular_vel)

    def close_loop_rotatation(self):
        self.goal = self.rotate[-1]
        angular_error = calculate_rotation_error(self.pose.pose, self.goal)
        
        if abs(angular_error) < rp.angular_error_tolerance:
            self.reset_control()
            self.get_logger().info("path_complete")
            self.cmd_publisher.publish(String(data="path_complete"))
            return
            
        linear_vel = 0
        angular_vel = self.path_angular_pid.update([angular_error, self.pose.header.stamp])

        self.motor_controller.set_velocity(linear_vel, angular_vel)

    def open_loop_control(self):
        self.motor_controller.set_velocity(self.vel_req.linear.x, self.vel_req.angular.z, closed_loop=False)

    def look_far_for(self, pose, path):
        pose_array=np.array([pose.position.x, pose.position.y]) 
        goals_array=np.array([[goal[0], goal[1]] for goal in path])

        dist_squared=np.sum((goals_array-pose_array)**2, axis=1)
        closest_index=np.argmin(dist_squared)

        return path[ min(closest_index + 1, len(path) - 1) ]

    def cmd_vel_callback(self, msg):
        self.vel_req = msg

    def position_callback(self, msg):
        self.new_position = float32_multi_array_to_two_d_array(msg)
    
    def path_callback(self, msg):
        self.path = float32_multi_array_to_two_d_array(msg)
    
    def rotate_callback(self, msg):
        self.rotate = float32_multi_array_to_two_d_array(msg)

    def pose_callback(self, msg):
        self.pose = msg

    def reset_control(self):
        self.positioning_linear_pid.clear_history()
        self.positioning_angular_pid.clear_history()
        self.path_linear_pid.clear_history()
        self.path_angular_pid.clear_history()
        self.path = []
        self.new_position = []
        self.rotate = []
        self.motor_controller.set_velocity(0, 0)

    def destroy_node(self):
        self.motor_controller.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()

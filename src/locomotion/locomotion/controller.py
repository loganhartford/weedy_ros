#!/mnt/shared/weedy_ros/src/locomotion/locomotion/pwm_venv/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from locomotion.motor_control import MotorController
from locomotion.localization import LocalizationNode

from pid import PID_ctrl
from utilities import calculate_pos_error, calculate_velocity_error
from robot_params import max_linear_speed, max_angular_speed

class ControllerNode(Node):
    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2):
        super().__init__('Controller')
        self.cmd_vel_subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.cmd_pose_subscription = self.create_subscription(PoseStamped, '/cmd_pose', self.cmd_pose_callback, 10)

        self.cmd_publisher = self.create_publisher(String, '/cmd', 10)
        
        self.velocity_target = Twist()
        self.goal_pose = PoseStamped()

        self.motor_controller = MotorController()
        self.localization_node = LocalizationNode()

        self.linear_pid=PID_ctrl(klp, klv, kli)
        self.angular_pid=PID_ctrl(kap, kav, kai)
        self.control_timer = self.create_timer(0.01, self.control_loop)
        self.linear_error_tolerance = 0.01 # 1cm TODO: tune this
        self.angular_error_tolerance = 0.05 # rad TODO: tune this

    def cmd_vel_callback(self, msg):
        self.velocity_target = msg
        # End control if the velocity target is 0
        if self.velocity_target.linear.x == 0 and self.velocity_target.angular.z == 0:
            self.reset_control()

    # End control if the pose target is 0
    def cmd_pose_callback(self, msg):
        self.goal_pose = msg
        if self.goal_pose.pose.position.x == 0 and self.goal_pose.pose.position.y == 0 and self.goal_pose.pose.orientation.z == 0:
            self.reset_control()

    def control_loop(self):
        # We have a velocity target
        current_odom = self.localization_node.update_odometry()
        if self.velocity_target.linear.x != 0 or self.velocity_target.angular.z != 0:
            linear_error, angluar_error = calculate_velocity_error(self.last_odom.twist.twist, self.velocity_target)

        # We have a pose target
        elif self.goal_pose.pose.position.x != 0 or self.goal_pose.pose.position.y != 0 or self.goal_pose.pose.orientation.z != 0:
            linear_error, angluar_error = calculate_pos_error(self.last_odom.pose.pose, self.goal_pose.pose)

            # Check if we reached the goal
            if linear_error < self.linear_error_tolerance and angluar_error < self.angular_error_tolerance:
                self.reset_control()
                self.cmd_publisher.publish(String(data="goal_reached"))
                return
        # No target
        else:
            return

        stamp = self.last_odom.header.stamp
        linear_vel = self.linear_pid.update([linear_error, stamp])
        angular_vel = self.angular_pid.update([angluar_error, stamp])

        # Bound velocities
        if abs(linear_vel) > max_linear_speed:
            linear_vel = max_linear_speed if linear_vel > 0 else -max_linear_speed
        if abs(angular_vel) > max_angular_speed:
            angular_vel = max_angular_speed if angular_vel > 0 else -max_angular_speed
        
        # print(linear_vel)

        self.motor_controller.set_velocity(linear_vel, angular_vel)

    def reset_control(self):
        self.linear_pid.clear_history()
        self.angular_pid.clear_history()
        self.motocontroller.set_velocity(0, 0)

    def destroy_node(self):
        self.motor_controller.stop()
        self.localization_node.destroy_node()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
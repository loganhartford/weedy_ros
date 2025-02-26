#!/mnt/shared/weedy_ros/src/locomotion/locomotion/pwm_venv/bin/python3
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from custom_msgs.msg import Points

from locomotion.motor_control import MotorController
from locomotion.pid import PID_ctrl
from utils.utilities import calculate_linear_error, calculate_angular_error
import utils.robot_params as rp


class ControllerNode(Node):
    def __init__(
        self,
        klp=5.0, kld=0.0, kli=2.0,
        kap=1.2, kad=0.0, kai=1.0,
    ):
        super().__init__('controller')

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(PoseStamped, '/pose', self.pose_callback, 1)
        self.create_subscription(Points, '/goal', self.goal_callback, 10)
        self.cmd_publisher = self.create_publisher(String, '/cmd', 10)

        self.pose = None
        self.vel_req = Twist()
        self.goal_list = None

        self.motor_controller = MotorController()
        self.linear_pid = PID_ctrl(
            klp, kld, kli,
            log_file="/mnt/shared/weedy_ros/src/locomotion/locomotion/outputs/lin_pid_log.csv"
        )
        self.angular_pid = PID_ctrl(
            kap, kad, kai,
            log_file="/mnt/shared/weedy_ros/src/locomotion/locomotion/outputs/ang_pid_log.csv"
        )

        self.control_timer = self.create_timer(0.01, self.control_loop)

        self.get_logger().info("Controller Initialized")
       
    def control_loop(self):
        if self.pose is None:
            return

        if self.goal_list == None:
            self.open_loop_control()
        else:
            self.close_loop_control()

    def close_loop_control(self):
        # Compute linear error to final goal and angular error to next goal.
        linear_error = calculate_linear_error(self.pose.pose, self.goal_list[-1])
        angular_error = calculate_angular_error(self.pose.pose, self.look_far_for(self.pose.pose, self.goal_list))

        # If the goal is reached, reset control.
        # if linear_error < rp.pid_linear_error_tolerance and angular_error < rp.angular_error_tolerance:
        if linear_error < rp.pid_linear_error_tolerance:
            self.reset_control()
            self.get_logger().info("goal_reached")
            self.cmd_publisher.publish(String(data="goal_reached"))
            return

        linear_vel = self.linear_pid.update([linear_error, self.pose.header.stamp])
        angular_vel = self.angular_pid.update([angular_error, self.pose.header.stamp])

        self.motor_controller.set_velocity(linear_vel, angular_vel)
        return

    def open_loop_control(self):
        self.motor_controller.set_velocity(self.vel_req.linear.x, self.vel_req.angular.z, closed_loop=False)

    def look_far_for(self, pose, goal_list):
        pose_array=np.array([pose.position.x, pose.position.y]) 
        goals_array=np.array([[goal[0], goal[1]] for goal in goal_list])

        dist_squared=np.sum((goals_array-pose_array)**2, axis=1)
        closest_index=np.argmin(dist_squared)

        return goal_list[ min(closest_index + 1, len(goal_list) - 1) ]

    def cmd_vel_callback(self, msg):
        self.vel_req = msg

    def goal_callback(self, msg):
        self.goal_list = msg.points

        self.goal_list = [[self.goal_list[i].x, self.goal_list[i].y, self.goal_list[i].z] for i in range(len(self.goal_list))]

    def pose_callback(self, msg):
        self.pose = msg

    def reset_control(self):
        self.linear_pid.clear_history()
        self.angular_pid.clear_history()
        self.goal_list = None
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

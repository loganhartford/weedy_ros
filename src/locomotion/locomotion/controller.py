#!/mnt/shared/weedy_ros/src/locomotion/locomotion/pwm_venv/bin/python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from custom_msgs.msg import Points

from locomotion.motor_control import MotorController
from locomotion.pid import PID_ctrl
from utils.utilities import calculate_pose_error
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
        self.goal = []
        self.alpha = 0.5  # LPF
        self.last_linear_velocity = 0.0
        self.last_angular_velocity = 0.0


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

        if self.goal == []:
            self.open_loop_control()
        else:
            self.close_loop_control()

    def close_loop_control(self):
        linear_error, angular_error = calculate_pose_error(self.pose.pose, self.goal[-1])

        # If the goal is reached, reset control.
        if linear_error < rp.pid_linear_error_tolerance and angular_error < rp.angular_error_tolerance:
            self.reset_control()
            self.get_logger().info("goal_reached")
            self.cmd_publisher.publish(String(data="goal_reached"))
            return

        linear_vel = self.linear_pid.update([linear_error, self.pose.header.stamp])
        angular_vel = self.angular_pid.update([angular_error, self.pose.header.stamp])

        self.motor_controller.set_velocity(linear_vel, angular_vel)
        return

    def open_loop_control(self):
        target_linear_vel = self.vel_req.linear.x
        target_angular_vel = self.vel_req.angular.z

        smooth_linear_vel = self.alpha * target_linear_vel + (1 - self.alpha) * self.last_linear_velocity
        smooth_angular_vel = self.alpha * target_angular_vel + (1 - self.alpha) * self.last_angular_velocity

        self.last_linear_velocity = smooth_linear_vel
        self.last_angular_velocity = smooth_angular_vel

        # Bound and round velocities.
        smooth_linear_vel = max(-rp.max_linear_speed, min(smooth_linear_vel, rp.max_linear_speed))
        if abs(smooth_angular_vel) > rp.max_zero_angular_speed:
            smooth_angular_vel = max(-rp.max_angular_speed, min(smooth_angular_vel, rp.max_angular_speed))
        smooth_linear_vel = round(smooth_linear_vel, 2)
        smooth_angular_vel = round(smooth_angular_vel, 2)

        self.motor_controller.set_velocity(smooth_linear_vel, smooth_angular_vel)

    def cmd_vel_callback(self, msg):
        self.vel_req = msg

    def goal_callback(self, msg):
        self.goal = msg.points

    def pose_callback(self, msg):
        self.pose = msg

    def reset_control(self):
        self.linear_pid.clear_history()
        self.angular_pid.clear_history()
        self.goal = []
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

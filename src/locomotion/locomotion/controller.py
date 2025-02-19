#!/mnt/shared/weedy_ros/src/locomotion/locomotion/pwm_venv/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from locomotion.motor_control import MotorController
from locomotion.localization import Localization
from locomotion.pid import PID_ctrl
from utils.utilities import calculate_pos_error
from utils.robot_params import (
    max_linear_speed,
    max_angular_speed,
    pid_linear_error_tolerance,
    log,
    max_zero_angular_speed,
)


class ControllerNode(Node):
    """
    ROS2 node for robot locomotion control using PID and open-loop control.
    Subscribes to target velocities and odometry goals, and publishes control commands.
    """
    def __init__(
        self,
        klp=5.0, kld=0.0, kli=2.0,
        kap=1.2, kad=0.0, kai=1.0,
        log_file="/mnt/shared/weedy_ros/src/locomotion/locomotion/outputs/pose_log.csv"
    ):
        super().__init__('controller')

        # Subscriptions
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, '/cmd_odom', self.cmd_odom_callback, 10)
        self.create_subscription(String, '/cmd', self.cmd_callback, 10)

        # Publishers
        self.cmd_publisher = self.create_publisher(String, '/cmd', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        # Control variables
        self.goal_odom = Odometry()
        self.velocity_target = Twist()
        self.last_linear_velocity = 0.0
        self.last_angular_velocity = 0.0
        self.alpha = 0.5  # Low-pass filter weight for velocity smoothing

        # Controllers and localization
        self.motor_controller = MotorController()
        self.localization = Localization()
        self.linear_pid = PID_ctrl(
            klp, kld, kli,
            log_file="/mnt/shared/weedy_ros/src/locomotion/locomotion/outputs/lin_pid_log.csv"
        )
        self.angular_pid = PID_ctrl(
            kap, kad, kai,
            log_file="/mnt/shared/weedy_ros/src/locomotion/locomotion/outputs/ang_pid_log.csv"
        )

        # Timer for the control loop
        self.control_timer = self.create_timer(0.01, self.control_loop)

        # Error tolerances
        self.linear_error_tolerance = pid_linear_error_tolerance
        self.angular_error_tolerance = 0.1  # rad (unused for now)

        # Setup pose logging if enabled
        if log:
            self.log_file = log_file
            self.setup_logger()

        self.get_logger().info("Controller Init Complete")

    def cmd_vel_callback(self, msg: Twist):
        """Update target velocity from /cmd_vel messages."""
        self.velocity_target = msg

    def cmd_odom_callback(self, msg: Odometry):
        """Update goal odometry from /cmd_odom messages and reset control if goal is zero."""
        self.goal_odom = msg
        if (self.goal_odom.pose.pose.position.x == 0 and
            self.goal_odom.pose.pose.position.y == 0 and
            self.goal_odom.pose.pose.orientation.z == 0):
            self.reset_control()

    def control_loop(self):
        """
        Main control loop that:
          - Updates current odometry.
          - Logs pose if enabled.
          - Uses closed-loop control if a goal pose is set.
          - Otherwise, performs open-loop velocity control with smoothing.
        """
        # Skip control if there is no active command or goal and no prior motion.
        if (self.goal_odom.pose.pose.position.x == 0 and
            self.goal_odom.pose.pose.position.y == 0 and
            self.goal_odom.pose.pose.orientation.z == 0 and
            self.velocity_target.linear.x == 0 and
            self.velocity_target.angular.z == 0 and
            self.last_linear_velocity == 0 and
            self.last_angular_velocity == 0):
            return

        try:
            current_odom = self.localization.update_odometry()
        except Exception as e:
            self.get_logger().error(f"Error updating odometry: {e}")
            return

        if current_odom is None:
            return

        if log:
            self.log_pose(current_odom)

        self.odom_publisher.publish(current_odom)

        # Closed-loop control: if a goal pose is specified, compute and apply corrections.
        if (self.goal_odom.pose.pose.position.x != 0 or
            self.goal_odom.pose.pose.position.y != 0 or
            self.goal_odom.pose.pose.orientation.z != 0):

            linear_error, angular_error = calculate_pos_error(
                current_odom.pose.pose, self.goal_odom.pose.pose
            )

            # If the goal is reached, reset control.
            if linear_error < self.linear_error_tolerance:
                self.reset_control()
                self.get_logger().info("goal_reached")
                self.cmd_publisher.publish(String(data="goal_reached"))
                return

            stamp = current_odom.header.stamp
            linear_vel = self.linear_pid.update([linear_error, stamp])
            angular_vel = 0.0  # Angular control is disabled for now

            # Bound velocities within robot limits.
            linear_vel = max(-max_linear_speed, min(linear_vel, max_linear_speed))
            angular_vel = max(-max_angular_speed, min(angular_vel, max_angular_speed))

            self.motor_controller.set_velocity(linear_vel, angular_vel)
            return

        # Open-loop control with low-pass filtering.
        target_linear_vel = self.velocity_target.linear.x
        target_angular_vel = self.velocity_target.angular.z

        smooth_linear_vel = self.alpha * target_linear_vel + (1 - self.alpha) * self.last_linear_velocity
        smooth_angular_vel = self.alpha * target_angular_vel + (1 - self.alpha) * self.last_angular_velocity

        self.last_linear_velocity = smooth_linear_vel
        self.last_angular_velocity = smooth_angular_vel

        # Bound and round velocities.
        smooth_linear_vel = max(-max_linear_speed, min(smooth_linear_vel, max_linear_speed))
        if abs(smooth_angular_vel) > max_zero_angular_speed:
            smooth_angular_vel = max(-max_angular_speed, min(smooth_angular_vel, max_angular_speed))
        smooth_linear_vel = round(smooth_linear_vel, 2)
        smooth_angular_vel = round(smooth_angular_vel, 2)

        self.motor_controller.set_velocity(smooth_linear_vel, smooth_angular_vel)

    def reset_control(self):
        """Reset PID histories, clear goal, and stop the motors."""
        self.linear_pid.clear_history()
        self.angular_pid.clear_history()
        self.goal_odom.pose.pose.position.x = 0
        self.goal_odom.pose.pose.position.y = 0
        self.goal_odom.pose.pose.orientation.z = 0
        self.motor_controller.set_velocity(0, 0)

    def setup_logger(self):
        """Initialize the pose log file with a header."""
        with open(self.log_file, "w") as file:
            file.write("Timestamp,X,Y,Z,Orientation_Z,Orientation_W\n")

    def log_pose(self, current_odom: Odometry):
        """Append the current pose to the log file as a CSV row."""
        pose = current_odom.pose.pose
        pos = pose.position
        orient = pose.orientation
        timestamp = Time.from_msg(current_odom.header.stamp).nanoseconds / 1e9
        with open(self.log_file, "a") as file:
            file.write(f"{timestamp},{pos.x},{pos.y},{pos.z},{orient.z},{orient.w}\n")

    def cmd_callback(self, msg: String):
        """Handle miscellaneous commands."""
        if msg.data == "reset_odom":
            new_odom = self.localization.reset_odometry()
            self.odom_publisher.publish(new_odom)

    def destroy_node(self):
        """Stop the motors and perform cleanup."""
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

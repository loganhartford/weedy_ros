from nav_msgs.msg import Odometry
from rclpy.clock import Clock
import math

from utils.robot_params import wheel_radius, wheel_base, ticks_per_revolution, log
from utils.uart import UART


class Localization:
    """
    Odometry estimation for a four-wheeled robot with front-wheel drive.
    The front wheels are driven, and back wheels are passive.
    This class computes the robot's pose from encoder ticks obtained via UART.
    """

    def __init__(self):
        # Robot parameters
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.ticks_per_revolution = ticks_per_revolution

        # Initial pose (meters, radians)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_ticks_left = None
        self.last_ticks_right = None

        # Clock and last time stamp for delta time computation
        self.clock = Clock()
        self.last_time = self.clock.now()

        # UART interface to get encoder ticks
        self.uart = UART()

        # Setup tick logging if enabled
        if log:
            self.log_file = "/mnt/shared/weedy_ros/src/locomotion/locomotion/outputs/ticks_log.csv"
            with open(self.log_file, "w") as file:
                file.write("ticks_left,ticks_right\n")

    def _adjust_ticks(self, delta, max_ticks, min_ticks):
        """
        Adjusts tick difference for rollover.
        """
        if delta > max_ticks / 2:
            return delta - (max_ticks - min_ticks + 1)
        elif delta < min_ticks / 2:
            return delta + (max_ticks - min_ticks + 1)
        return delta

    def update_odometry(self):
        """
        Retrieves new encoder ticks, computes the change in pose (x, y, theta),
        and returns a ROS Odometry message containing the updated state.
        """
        try:
            ticks_left, ticks_right, stamp = self.uart.get_ticks()
        except Exception as e:
            raise e

        # On the first run, initialize previous tick values and timestamp.
        if self.last_ticks_left is None or self.last_ticks_right is None:
            self.last_ticks_left = ticks_left
            self.last_ticks_right = ticks_right
            self.last_time = stamp
            return None

        # Define tick rollover limits.
        max_ticks = 2**15 - 1
        min_ticks = -2**15

        # Compute raw differences in ticks.
        delta_left = ticks_left - self.last_ticks_left
        delta_right = ticks_right - self.last_ticks_right

        # Adjust for rollover.
        delta_left = self._adjust_ticks(delta_left, max_ticks, min_ticks)
        delta_right = self._adjust_ticks(delta_right, max_ticks, min_ticks)

        self.last_ticks_left = ticks_left
        self.last_ticks_right = ticks_right

        if log:
            with open(self.log_file, "a") as file:
                file.write(f"{ticks_left},{ticks_right}\n")

        # Compute time elapsed
        delta_time = (stamp - self.last_time).nanoseconds * 1e-9
        self.last_time = stamp

        # Invert left tick delta (assumes left encoder counts down when moving forward).
        effective_delta_left = -delta_left
        effective_delta_right = delta_right

        # Compute wheel displacements.
        d_left = (effective_delta_left / self.ticks_per_revolution) * (2 * math.pi * self.wheel_radius)
        d_right = (effective_delta_right / self.ticks_per_revolution) * (2 * math.pi * self.wheel_radius)

        # Average linear displacement and change in heading.
        d = (d_left + d_right) / 2.0
        delta_theta = (d_right - d_left) / self.wheel_base

        # Update pose using midpoint integration.
        self.x += d * math.cos(self.theta + delta_theta / 2.0)
        self.y += d * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta

        # Normalize theta to the range [-pi, pi].
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        # Compute velocities.
        linear_velocity = d / delta_time
        angular_velocity = delta_theta / delta_time

        # Create and populate the Odometry message.
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Pose (storing yaw in orientation.z for debugging; typically use a quaternion).
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation.z = self.theta

        # Twist
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity

        return odom_msg

    def reset_odometry(self):
        """
        Resets the estimated pose to zero and returns the updated odometry.
        """
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        return self.update_odometry()

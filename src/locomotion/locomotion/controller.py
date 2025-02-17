#!/mnt/shared/weedy_ros/src/locomotion/locomotion/pwm_venv/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from locomotion.motor_control import MotorController
from locomotion.localization import Localization

import locomotion.plot
from locomotion.pid import PID_ctrl
from utils.utilities import calculate_pos_error
from utils.robot_params import max_linear_speed, max_angular_speed, pid_linear_error_tolerance, log, max_zero_angular_speed

class ControllerNode(Node):
    # TODO: Tune angluar with second motor
    # TODO: Tune both on real robot
    def __init__(self, klp=5.0, kld=0.0, kli=2.0, kap=1.2, kad=0.0, kai=1.0,log_file="/mnt/shared/weedy_ros/src/locomotion/locomotion/outputs/pose_log.csv"):
        super().__init__('controller')
        self.cmd_vel_subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.cmd_odom_subscription = self.create_subscription(Odometry, '/cmd_odom', self.cmd_odom_callback, 10)
        self.cmd_subscription = self.create_subscription(String, '/cmd', self.cmd_callback, 10)

        self.cmd_publisher = self.create_publisher(String, '/cmd', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        
        self.goal_odom = Odometry()
        self.velocity_target = Twist()
        self.last_linear_velocity = 0.0
        self.last_angular_velocity = 0.0
        self.alpha = 0.5  # TODO: tune for how agressively the robot should stop

        self.motor_controller = MotorController()
        self.localization = Localization()

        self.linear_pid=PID_ctrl(klp, kld, kli, log_file="/mnt/shared/weedy_ros/src/locomotion/locomotion/outputs/lin_pid_log.csv")
        self.angular_pid=PID_ctrl(kap, kad, kai, log_file="/mnt/shared/weedy_ros/src/locomotion/locomotion/outputs/ang_pid_log.csv")
        self.control_timer = self.create_timer(0.01, self.control_loop)
        self.linear_error_tolerance = pid_linear_error_tolerance
        self.angular_error_tolerance = 0.1 # rad TODO: tune this

        if log:
            self.log_file = log_file
            self.setup_logger()
        
        self.get_logger().info("Controller Init Complete")
    
    def cmd_vel_callback(self, msg):
        self.velocity_target = msg

    # End control if the pose target is 0
    def cmd_odom_callback(self, msg):
        self.goal_odom = msg
        if self.goal_odom.pose.pose.position.x == 0 and self.goal_odom.pose.pose.position.y == 0 and self.goal_odom.pose.pose.orientation.z == 0:
            self.reset_control()

    def control_loop(self):
        if (self.goal_odom.pose.pose.position.x == 0 and self.goal_odom.pose.pose.position.y == 0 and self.goal_odom.pose.pose.orientation.z == 0) and (self.velocity_target.linear.x == 0 and self.velocity_target.angular.z == 0) and (self.last_angular_velocity == 0 and self.last_linear_velocity == 0):
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

        # We have a pose target
        if self.goal_odom.pose.pose.position.x != 0 or self.goal_odom.pose.pose.position.y != 0 or self.goal_odom.pose.pose.orientation.z != 0:
            
            # self.get_logger().info(f"Goal: {self.goal_odom.pose.pose.position.x}, Curent: {current_odom.pose.pose.position.x}")

            linear_error, angular_error = calculate_pos_error(current_odom.pose.pose, self.goal_odom.pose.pose)

            # Check if we reached the goal
            if linear_error < self.linear_error_tolerance:
                self.reset_control()
                self.get_logger().info(f"goal_reached")
                self.cmd_publisher.publish(String(data="goal_reached"))
                return
        
            stamp = current_odom.header.stamp
            linear_vel = self.linear_pid.update([linear_error, stamp])
            # angular_vel = self.angular_pid.update([angular_error, stamp])
            angular_vel = 0.0 # no angular control for now

            # Bound velocities
            if abs(linear_vel) > max_linear_speed:
                linear_vel = max_linear_speed if linear_vel > 0 else -max_linear_speed
            if abs(angular_vel) > max_angular_speed:
                angular_vel = max_angular_speed if angular_vel > 0 else -max_angular_speed
        
            self.motor_controller.set_velocity(linear_vel, angular_vel)
            return
        
        # Open loop velocity control with gain scheduling
        target_linear_vel = self.velocity_target.linear.x
        target_angular_vel = self.velocity_target.angular.z

        # LPF
        smooth_linear_vel = self.alpha * target_linear_vel + (1 - self.alpha) * self.last_linear_velocity
        smooth_angular_vel = self.alpha * target_angular_vel + (1 - self.alpha) * self.last_angular_velocity

        self.last_linear_velocity = smooth_linear_vel
        self.last_angular_velocity = smooth_angular_vel

        # Bound velocities to robot limits
        if abs(smooth_linear_vel) > max_linear_speed:
            smooth_linear_vel = max_linear_speed if smooth_linear_vel > 0 else -max_linear_speed
        # if abs(smooth_angular_vel) > max_angular_speed: 
        if abs(smooth_angular_vel) > max_zero_angular_speed: # zero point turns
            smooth_angular_vel = max_angular_speed if smooth_angular_vel > 0 else -max_angular_speed

        # This is so the motor will actually shut off since in motor control
        # we keep the motor runing if the requested vel is greater than 0
        smooth_linear_vel = round(smooth_linear_vel, 2)
        smooth_angular_vel = round(smooth_angular_vel, 2)

        self.motor_controller.set_velocity(smooth_linear_vel, smooth_angular_vel)
        
    def reset_control(self):
        self.linear_pid.clear_history()
        self.angular_pid.clear_history()
        self.goal_odom.pose.pose.position.x = 0
        self.goal_odom.pose.pose.position.y = 0
        self.goal_odom.pose.pose.orientation.z = 0
        self.motor_controller.set_velocity(0, 0)
    
    def setup_logger(self):
        with open(self.log_file, "w") as file:
            file.write("Timestamp,X,Y,Z,Orientation_Z,Orientation_W\n") 

    def log_pose(self, current_odom):
        pose = current_odom.pose.pose
        position = pose.position
        orientation = pose.orientation
        timestamp = Time.from_msg(current_odom.header.stamp).nanoseconds / 1e9

        # Write to CSV
        with open(self.log_file, "a") as file:
            file.write(f"{timestamp},{position.x},{position.y},{position.z},{orientation.z},{orientation.w}\n")

    def cmd_callback(self, msg):
        if msg.data == "reset_odom":
            self.x = 0.0  # m
            self.y = 0.0  # m
            self.theta = 0.0  # rad

    def destroy_node(self):
        self.motor_controller.stop()
        # plot.main()
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
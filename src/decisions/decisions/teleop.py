import pygame
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import os

# Fix for headless environments (SSH, no GUI)
os.environ["SDL_VIDEODRIVER"] = "dummy"

from utils.robot_params import max_linear_speed, max_angular_speed

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')

        pygame.init()

        # Detect joystick
        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No joystick detected. Ensure it's connected.")
            exit(1)

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f"Found joystick: {self.joystick.get_name()}")

        self.cmd_publisher = self.create_publisher(String, '/cmd', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Speed factors
        self.linear_speed = max_linear_speed   # m/s
        self.angular_speed = max_angular_speed  # rad/s

        self.last_linear = 0.0
        self.last_angular = 0.0

    def run(self):
        while rclpy.ok():
            pygame.event.pump()
            events = pygame.event.get()

            for event in events:
                twist = Twist()
                cmd = String()

                if event.type == pygame.JOYAXISMOTION:
                    # Left joystick: Axis 0 (X for turning), Axis 1 (Y for forward/backward)
                    linear_x = -self.joystick.get_axis(1) * self.linear_speed
                    angular_z = self.joystick.get_axis(0) * self.angular_speed

                    # Dead zone check to reduce redundant messages
                    if abs(linear_x - self.last_linear) > 0.01 or abs(angular_z - self.last_angular) > 0.01:
                        twist.linear.x = linear_x
                        twist.angular.z = angular_z
                        self.cmd_vel_publisher.publish(twist)
                        self.last_linear = linear_x
                        self.last_angular = angular_z

                elif event.type == pygame.JOYBUTTONDOWN:
                    button = event.button

                    if button == 0:  # A Button
                        self.get_logger().info('A Button pressed')

                    elif button == 1:  # B Button
                        self.get_logger().info('B Button pressed')

                    elif button == 2:  # X Button - Capture Image
                        cmd.data = "get_img"
                        self.get_logger().info("Capturing image.")
                        self.cmd_publisher.publish(cmd)

                    elif button == 3:  # Y Button - Emergency Stop
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        cmd.data = "stop"
                        self.get_logger().error('Emergency STOP!')
                        self.cmd_publisher.publish(cmd)
                        self.cmd_vel_publisher.publish(twist)
            
            # Allow ROS2 to check for shutdown signals
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        pygame.quit()


if __name__ == '__main__':
    main()

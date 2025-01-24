import pygame
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

# Fix for headless environments (SSH, no GUI)
import os
os.environ["SDL_VIDEODRIVER"] = "dummy"

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')

        # Initialize pygame
        pygame.init()

        # Detect joystick
        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No joystick detected. Ensure it's connected.")
            exit(1)

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f"Found joystick: {self.joystick.get_name()}")

        # ROS2 publishers
        self.cmd_publisher = self.create_publisher(String, '/cmd', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Speed factors
        self.linear_speed = 0.3    # m/s
        self.angular_speed = 0.5   # rad/s

        # Joystick state
        self.vert = 0
        self.hor = 0

        # Button debounce state
        self.last_button_press_time = [0] * self.joystick.get_numbuttons()
        self.debounce_time = 0.3  # 300 ms debounce time

        # Main loop
        self.timer = self.create_timer(0.05, self.read_inputs)  # 20Hz update rate

    def read_inputs(self):
        pygame.event.pump()  # Process event queue

        twist = Twist()
        cmd = String()

        # Left joystick controls movement
        self.hor = self.joystick.get_axis(0)  # X-axis (turning)
        self.vert = -self.joystick.get_axis(1)  # Y-axis (forward/backward, inverted)

        twist.linear.x = self.vert * self.linear_speed
        twist.angular.z = self.hor * self.angular_speed
        self.cmd_vel_publisher.publish(twist)

        current_time = time.time()

        # Button presses with debounce
        if self.joystick.get_button(0) and current_time - self.last_button_press_time[0] > self.debounce_time:  # A Button
            self.last_button_press_time[0] = current_time
            self.get_logger().info('A Button pressed')
        elif self.joystick.get_button(1) and current_time - self.last_button_press_time[1] > self.debounce_time:  # B Button
            self.last_button_press_time[1] = current_time
            self.get_logger().info('B Button pressed')
        elif self.joystick.get_button(2) and current_time - self.last_button_press_time[2] > self.debounce_time:  # X Button - Capture Image
            self.last_button_press_time[2] = current_time
            cmd.data = "get_img"
            self.get_logger().info("Capturing image.")
            self.cmd_publisher.publish(cmd)
        elif self.joystick.get_button(3) and current_time - self.last_button_press_time[3] > self.debounce_time:  # Y Button - Emergency Stop
            self.last_button_press_time[3] = current_time
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            cmd.data = "stop"
            self.get_logger().error('Emergency STOP!')
            self.cmd_publisher.publish(cmd)
            self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        pygame.quit()

if __name__ == '__main__':
    main()

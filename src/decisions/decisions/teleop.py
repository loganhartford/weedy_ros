import os
import pygame
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, UInt8MultiArray, Bool

from utils.nucleo_gpio import NucleoGPIO
import utils.robot_params as rp
from utils.utilities import package_removal_command

# Fix for headless environments (SSH, no GUI)
os.environ["SDL_VIDEODRIVER"] = "dummy"


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')

        pygame.init()
        self.joystick = None
        self.create_timer(0.1, self.timer_callback)
        self.create_timer(1.0, self.check_joystick_connection)

        self.cmd_pub = self.create_publisher(String, '/cmd', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.uart_pub = self.create_publisher(UInt8MultiArray, "/send_uart", 10)
        self.reset_odom_pub = self.create_publisher(Bool, "/reset_odom", 10)
        self.pause_path_pub = self.create_publisher(String, "/ctr_cmd", 10)

        self.last_linear = 0.0
        self.last_angular = 0.0

        self.gpio = NucleoGPIO()

    def check_joystick_connection(self):
        if self.joystick is None or not self.joystick.get_init():
            pygame.joystick.quit()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self.get_logger().info(f"Remote connected: {self.joystick.get_name()}")
            else:
                self.get_logger().warn("No remote detected. Retrying...")

    def timer_callback(self):
        if not self.joystick or not self.joystick.get_init():
            return

        pygame.event.pump()
        for event in pygame.event.get():
            self.get_logger().debug(f"Event: {event}")
            if event.type == pygame.JOYAXISMOTION:
                self.handle_axis_motion(event)
            elif event.type == pygame.JOYHATMOTION:
                self.handle_hat_motion(event)
            elif event.type == pygame.JOYBUTTONDOWN:
                self.handle_button_down(event)
            elif event.type == pygame.JOYBUTTONUP and event.button == 10:  # Button 10 - Drill release
                self.send_manual_command(rp.stop_byte)

    def handle_axis_motion(self, event):
        twist = Twist()
        linear_x, angular_z = 0.0, 0.0

        if event.axis in {2, 5}:  # Triggers
            value = self.joystick.get_axis(event.axis) + 1
            if event.axis == 2 and value > 0.5:
                angular_z = rp.max_angular_speed
            elif event.axis == 5 and value > 0.5:
                angular_z = -rp.max_angular_speed
            else:
                angular_z = 0.0
        else:  # Joystick movement (Left stick: linear, Right stick: angular)
            linear_x = -self.joystick.get_axis(1) * rp.max_linear_speed  # Left stick Y-axis (forward/backward)
            angular_z = -self.joystick.get_axis(0) * rp.max_angular_speed  # Left stick X-axis (rotation)

        if abs(linear_x - self.last_linear) > 0.01 or abs(angular_z - self.last_angular) > 0.01:
            twist.linear.x, twist.angular.z = linear_x, angular_z
            self.cmd_vel_pub.publish(twist)
            self.last_linear, self.last_angular = linear_x, angular_z

    def handle_hat_motion(self, event):
        direction = {(-1, 0): rp.left_byte, (1, 0): rp.right_byte, (0, 1): rp.up_byte, (0, -1): rp.down_byte}.get(event.value, rp.stop_byte)
        self.send_manual_command(direction)

    def handle_button_down(self, event):
        cmd = String()
        twist = Twist()

        # Button mapping (based on Xbox controller layout)
        commands = {
            0: "start",      # A button
            1: "battery",      # B button
            2: "get_img",      # X button
            3: "stop",         # Y button (Emergency stop)
            4: "print_pose",   # LB button
        }

        if event.button in commands:
            cmd.data = commands[event.button]
            if event.button == 3:  # Emergency Stop
                self.get_logger().error("Emergency STOP!")
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.pause_path_pub.publish(String(data="stop"))
        elif event.button == 5:  # RB button - Reset Odometry
            self.reset_odom_pub.publish(Bool(data=True))
        elif event.button == 6:  # Back button (-) - Toggle GPIO reset
            self.toggle_nucleo_reset()
        elif event.button == 7:  # Start button (+) - Removal
            # Homography alignement positions
            self.uart_pub.publish(package_removal_command(23))
            # self.uart_pub.publish(package_removal_command(196))
        elif event.button == 10:  # Right Stick Press - Drill activation (uses manual_control)
            self.send_manual_command(rp.drill_byte)

        if cmd.data:
            self.cmd_pub.publish(cmd)
            self.cmd_vel_pub.publish(twist)

    def send_manual_command(self, command):
        msg = UInt8MultiArray()
        msg.data = [command]
        self.uart_pub.publish(msg)

    def toggle_nucleo_reset(self):
        try:
            self.gpio.toggle_reset()
        except Exception as e:
            self.get_logger().error(f"Error toggling reset: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        pygame.quit()


if __name__ == '__main__':
    main()

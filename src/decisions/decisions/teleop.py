import os
import pygame
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from utils.uart import UART
from utils.nucleo_gpio import NucleoGPIO
from utils.robot_params import max_linear_speed, max_angular_speed, max_zero_angular_speed

# Fix for headless environments (SSH, no GUI)
os.environ["SDL_VIDEODRIVER"] = "dummy"


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')

        pygame.init()
        self.joystick = None
        self.create_timer(0.1, self.timer_callback)
        self.create_timer(1.0, self.check_joystick_connection)

        # Publishers
        self.cmd_publisher = self.create_publisher(String, '/cmd', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Speed factors
        self.linear_speed = max_linear_speed
        self.angular_speed = max_angular_speed

        self.last_linear = 0.0
        self.last_angular = 0.0

        self.uart = UART()
        self.gpio = NucleoGPIO()

    def check_joystick_connection(self):
        """Checks if a joystick is connected and initializes it if found."""
        if self.joystick is None or not self.joystick.get_init():
            pygame.joystick.quit()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self.get_logger().info(f"Joystick connected: {self.joystick.get_name()}")
            else:
                self.get_logger().warn("No joystick detected. Retrying...")

    def timer_callback(self):
        """Processes joystick input and publishes commands."""
        if not self.joystick or not self.joystick.get_init():
            return

        pygame.event.pump()
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                self.handle_axis_motion(event)
            elif event.type == pygame.JOYHATMOTION:
                self.handle_hat_motion(event)
            elif event.type == pygame.JOYBUTTONDOWN:
                self.handle_button_down(event)
            elif event.type == pygame.JOYBUTTONUP and event.button == 10:  # Button 10 - Drill release
                self.send_manual_command("stop")

    def handle_axis_motion(self, event):
        """Handles joystick axis movements for velocity control."""
        twist = Twist()
        linear_x, angular_z = 0.0, 0.0

        if event.axis in {2, 5}:  # Triggers (LT / RT)
            value = self.joystick.get_axis(event.axis) + 1
            if event.axis == 2 and value > 0.5:
                angular_z = -max_zero_angular_speed
            elif event.axis == 5 and value > 0.5:
                angular_z = max_zero_angular_speed
            else:
                angular_z = 0.0
        else:  # Joystick movement (Left stick: linear, Right stick: angular)
            linear_x = -self.joystick.get_axis(1) * self.linear_speed  # Left stick Y-axis (forward/backward)
            angular_z = -self.joystick.get_axis(0) * self.angular_speed  # Left stick X-axis (rotation)

        if abs(linear_x - self.last_linear) > 0.01 or abs(angular_z - self.last_angular) > 0.01:
            twist.linear.x, twist.angular.z = linear_x, angular_z
            self.cmd_vel_publisher.publish(twist)
            self.last_linear, self.last_angular = linear_x, angular_z

    def handle_hat_motion(self, event):
        """Handles D-pad directional input."""
        direction = {(-1, 0): "left", (1, 0): "right", (0, 1): "up", (0, -1): "down"}.get(event.value, "stop")
        self.send_manual_command(direction)

    def handle_button_down(self, event):
        """Handles joystick button presses and sends corresponding commands."""
        cmd = String()
        twist = Twist()

        # Button mapping (based on Xbox controller layout)
        commands = {
            0: "circuit",        # A button
            1: "battery",      # B button
            2: "get_img",      # X button
            3: "stop",         # Y button (Emergency stop)
            4: "print_odom",   # LB button
        }

        if event.button in commands:
            cmd.data = commands[event.button]
            if event.button == 3:  # Emergency Stop
                self.get_logger().error("Emergency STOP!")
                twist.linear.x = 0.0
                twist.angular.z = 0.0
        elif event.button == 5:  # RB button - Send UART command (uses send_command)
            self.send_uart_command(1, 50)
        elif event.button == 6:  # Back button (-) - Toggle GPIO reset
            self.toggle_gpio_reset()
        elif event.button == 7:  # Start button (+) - Circle turn
            radius = 0.35  # Choose a radius greater than 0.269 m for pure forward motion
            twist.linear.x = 0.3
            twist.angular.z = -twist.linear.x / radius
            self.cmd_vel_publisher.publish(twist)
        elif event.button == 10:  # Right Stick Press - Drill activation (uses manual_control)
            self.send_manual_command("drill")

        if cmd.data:
            self.cmd_publisher.publish(cmd)
            self.cmd_vel_publisher.publish(twist)

    def send_manual_command(self, command):
        """Sends manual control commands via UART (used for joystick D-pad and drill)."""
        try:
            self.uart.manual_control(command)
        except Exception as e:
            self.get_logger().error(f"Error sending manual command: {e}")

    def send_uart_command(self, arg1, arg2):
        """Sends UART commands via send_command() (used for button presses)."""
        try:
            self.uart.send_command(arg1, arg2, wait=False)
        except Exception as e:
            self.get_logger().error(f"Error sending UART command: {e}")

    def toggle_gpio_reset(self):
        """Toggles GPIO reset."""
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

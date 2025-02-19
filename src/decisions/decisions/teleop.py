import os
import pygame
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from utils.uart import UART
from utils.nucleo_gpio import NucleoGpio

# Fix for headless environments (SSH, no GUI)
os.environ["SDL_VIDEODRIVER"] = "dummy"

from utils.robot_params import max_linear_speed, max_angular_speed, max_zero_angular_speed

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')

        pygame.init()
        self.joystick = None
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.joystick_timer = self.create_timer(1.0, self.check_joystick_connection)

        # Publishers
        self.cmd_publisher = self.create_publisher(String, '/cmd', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Speed factors
        self.linear_speed = max_linear_speed   # m/s
        self.angular_speed = max_angular_speed  # rad/s

        self.last_linear = 0.0
        self.last_angular = 0.0

        self.uart = UART()
        self.gpio = NucleoGpio()

    def check_joystick_connection(self):
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
        if self.joystick is None or not self.joystick.get_init():
            return  # Skip execution if no joystick is connected

        pygame.event.pump()  # Process pygame events
        events = pygame.event.get()  # Get all events since last call

        for event in events:
            twist = Twist()
            cmd = String()

            if event.type == pygame.JOYAXISMOTION:
                if event.axis == 2:
                    left_trigger = self.joystick.get_axis(2) + 1
                    angular_z = -max_zero_angular_speed if left_trigger > 0.5 else 0.0
                    linear_x = 0.0
                elif event.axis == 5:
                    right_trigger = self.joystick.get_axis(5) + 1
                    angular_z = max_zero_angular_speed if right_trigger > 0.5 else 0.0
                    linear_x = 0.0
                else:
                    linear_x = -self.joystick.get_axis(1) * self.linear_speed
                    angular_z = -self.joystick.get_axis(0) * self.angular_speed

                if abs(linear_x - self.last_linear) > 0.01 or abs(angular_z - self.last_angular) > 0.01:
                    twist.linear.x = linear_x
                    twist.angular.z = angular_z
                    self.cmd_vel_publisher.publish(twist)
                    self.last_linear = linear_x
                    self.last_angular = angular_z

            elif event.type == pygame.JOYHATMOTION:
                direction = {(-1, 0): "left", (1, 0): "right", (0, 1): "up", (0, -1): "down"}.get(event.value, "stop")
                try:
                    self.uart.manual_control(direction)
                except Exception as e:
                    self.get_logger().error(f"Error sending command: {e}")

            elif event.type == pygame.JOYBUTTONDOWN:
                button = event.button
                if button == 0:
                    cmd.data = "start"
                elif button == 1:
                    cmd.data = "battery"
                elif button == 2:
                    cmd.data = "get_img"
                    self.get_logger().info("Capturing image.")
                elif button == 3:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    cmd.data = "stop"
                    self.get_logger().error("Emergency STOP!")
                elif button == 4:
                    cmd.data = "print_odom"
                    self.get_logger().info("Print odom.")
                elif button == 5:
                    try:
                        self.uart.send_command(1, 50)
                    except Exception as e:
                        self.get_logger().error(f"Error sending command: {e}")
                elif button == 6: # - button
                    try:
                        self.gpio.toggle_reset()
                    except Exception as e:
                        self.get_logger().error(f"Error toggling reset: {e}")
                elif button == 7: # + button
                    pass
                
                if cmd.data:
                    self.cmd_publisher.publish(cmd)
                    self.cmd_vel_publisher.publish(twist)

            elif event.type == pygame.JOYBUTTONDOWN and event.button == 10:
                try:
                    self.uart.manual_control("drill")
                except Exception as e:
                    self.get_logger().error(f"Error sending command: {e}")

            elif event.type == pygame.JOYBUTTONUP and event.button == 10:
                try:
                    self.uart.manual_control("stop")
                except Exception as e:
                    self.get_logger().error(f"Error sending command: {e}")


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

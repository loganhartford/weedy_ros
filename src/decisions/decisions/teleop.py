import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from evdev import InputDevice, ecodes
import evdev
import requests

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')

        # Automatically detect the controller
        self.controller = self.find_controller()
        if not self.controller:
            self.get_logger().error("Controller not found.")
            exit(1)

        self.cmd_publisher = self.create_publisher(String, '/cmd', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Simple way to control the robot
        self.linear_speed = -0.3    # m/s
        self.angular_speed = -0.5   # rad/s
        self.vert = 0
        self.hor = 0

        # Taking pictures
        self.image_url = "http://localhost:8000"

        self.read_inputs()

    def read_inputs(self):
        for event in self.controller.read_loop():
            twist = Twist()
            cmd = String()

            # Joystick movements
            # TODO: figure out how to control the robot with this
            if event.type == ecodes.EV_ABS:
                if event.code == 0:  # Left Stick X-axis (Turning)
                    pass
                elif event.code == 1:  # Left Stick Y-axis (Forward/Backward)
                    pass
                elif event.code == 17: # Up/down
                    self.vert = event.value
                elif event.code == 16: # Left/Right
                    self.hor = event.value
                
                twist.linear.x = self.vert * self.linear_speed
                twist.angular.z = self.hor * self.angular_speed

                self.cmd_vel_publisher.publish(twist)
            
            # Buton presses
            elif event.type == ecodes.EV_KEY and event.value == 1:
                if event.code == 304:  # A Button
                    print('A Button pressed')
                elif event.code == 305:  # B Button
                    print('B Button pressed')
                elif event.code == 307:  # X Button - Capture Image
                    cmd.data = "get_img"
                    self.get_logger().info("Capturing image.")
                elif event.code == 308:  # Y Button - Emergency Stop
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    cmd.data = "stop"
                    self.get_logger().error('Emergency STOP!')
                    self.cmd_publisher.publish(cmd)
                    self.cmd_vel_publisher.publish(twist)
            
            
    
    def find_controller(self):
        for device_path in evdev.list_devices():
            device = evdev.InputDevice(device_path)
            if "8BitDo Ultimate 2C Wireless Controller" in device.name:
                self.get_logger().info(f"Found controller: {device.name} at {device.path}")
                return device
        return None


def main(args=None):
    rclpy.init(args=args)
    try:
        node = TeleopNode()
    except FileNotFoundError:
        print("Controller not found. Check the path.")
        return
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

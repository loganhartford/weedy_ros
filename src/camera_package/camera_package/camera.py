import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import random

class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")
        self.subscription = self.create_subscription(Bool, "button_state", self.button_callback, 10)
        self.publisher_ = self.create_publisher(String, "led_command", 10)

    def button_callback(self, msg):
        if msg.data:  # If the button is pressed
            result = random.choice(["success", "error"])
            self.publisher_.publish(String(data=result))
            self.get_logger().info(f"Published {result} to led_command")

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

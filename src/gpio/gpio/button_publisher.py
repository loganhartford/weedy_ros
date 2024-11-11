import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import lgpio
import time

class ButtonPublisherNode(Node):
    def __init__(self):
        super().__init__("button_publisher")

        self.button_pin = 17
        self.chip = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_input(self.chip, self.button_pin)

        self.publisher_ = self.create_publisher(Bool, "/button_state", 10)
        
        self.monitor_button()

    def monitor_button(self):
        try:
            # Run until node is shutdown
            while rclpy.ok():
                # Wait for the button to be pressed (polling)
                while lgpio.gpio_read(self.chip, self.button_pin) == 0:
                    time.sleep(0.01)  # Debounce delay

                msg = Bool()
                msg.data = True
                self.publisher_.publish(msg)
                self.get_logger().info(f'Button pressed!')

                # Wait for the button to be released
                while lgpio.gpio_read(self.chip, self.button_pin) == 1:
                    time.sleep(0.01)  # Debounce delay

        except KeyboardInterrupt:
            self.get_logger().info("Shutting down button monitoring...")

    def destroy_node(self):
        lgpio.gpiochip_close(self.chip)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ButtonPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

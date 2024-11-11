import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import lgpio
import time

class LEDSubscriberNode(Node):
    def __init__(self):
        super().__init__("led_subscriber")
        self.green_led_pin = 22
        self.red_led_pin = 27
        self.chip = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.chip, self.green_led_pin)
        lgpio.gpio_claim_output(self.chip, self.red_led_pin)

        self.subscription = self.create_subscription(String, "/led_command", self.led_callback, 10)

    def led_callback(self, msg):
        if msg.data == "success":
            lgpio.gpio_write(self.chip, self.green_led_pin, 1)
            time.sleep(1)
            lgpio.gpio_write(self.chip, self.green_led_pin, 0)
        elif msg.data == "error":
            lgpio.gpio_write(self.chip, self.red_led_pin, 1)
            time.sleep(1)
            lgpio.gpio_write(self.chip, self.red_led_pin, 0)

    def destroy_node(self):
        lgpio.gpiochip_close(self.chip)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LEDSubscriberNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

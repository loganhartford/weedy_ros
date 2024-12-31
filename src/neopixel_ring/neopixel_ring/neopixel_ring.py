#!/mnt/shared/weedy_ros/src/neo_pixel/neo_pixel/venv/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from pi5neo import Pi5Neo

class NeoPixelNode(Node):
    def __init__(self):
        super().__init__("neopixel_ring")

        self.neo = Pi5Neo('/dev/spidev0.0', 16, 800)

        self.subscription = self.create_subscription(ColorRGBA, "/ring", self.led_callback, 10)
    

    def led_callback(self, msg):
        self.neo.fill_strip(int(msg.r * 255 * msg.a), int(msg.g * 255 * msg.a), int(msg.b * 255 * msg.a))
        self.neo.update_strip()

    def destroy_node(self):
        self.neo.clear_strip()
        self.neo.update_strip()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = NeoPixelNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

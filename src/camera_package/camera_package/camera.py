import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from PIL import Image as PILImage
from io import BytesIO
import requests
import numpy as np
from cv_bridge import CvBridge

class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")
        self.subscription = self.create_subscription(Bool, "button_state", self.button_callback, 10)
        self.subscription = self.create_subscription(Bool, "get_img", self.get_callback, 10)
        self.image_publisher = self.create_publisher(Image, "image_data", 10)
        self.led_command_publisher = self.create_publisher(String, "led_command", 10)
        self.image_url = "http://10.0.0.171:8000"
        self.bridge = CvBridge()

    def get_callback(self, msg):
        if msg.data:
            self.request_img()

    def button_callback(self, msg):
        if msg.data:
            self.request_img()
    
    def request_img(self):
        try:
            response = requests.get(self.image_url)
            response.raise_for_status()

            image = PILImage.open(BytesIO(response.content))
            image_np = np.array(image)[:, :, ::-1]

            ros_image = self.bridge.cv2_to_imgmsg(image_np, encoding="bgr8")
            self.image_publisher.publish(ros_image)
            self.led_command_publisher.publish(String(data="success"))
            self.get_logger().info("Image data published to image_data topic and success sent to led_command.")
        
        except requests.exceptions.RequestException as e:
            self.led_command_publisher.publish(String(data="error"))
            self.get_logger().error(f"Failed to fetch image: {e}")
    

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
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
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
        self.publisher_ = self.create_publisher(Image, "image_data", 10)
        self.image_url = "http://10.0.0.171:8000"  # Replace with your Raspberry Pi IP
        self.bridge = CvBridge()

    def button_callback(self, msg):
        if msg.data:  # Button pressed
            try:
                # Request and image
                response = requests.get(self.image_url)
                response.raise_for_status()

                # Open and convert the image to a numpy array
                image = PILImage.open(BytesIO(response.content))
                image_np = np.array(image)

                # Convert the image to a ROS Image message
                ros_image = self.bridge.cv2_to_imgmsg(image_np, encoding="rgb8")
                self.publisher_.publish(ros_image)
                self.get_logger().info("Image data published to image_data topic.")
            
            except requests.exceptions.RequestException as e:
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

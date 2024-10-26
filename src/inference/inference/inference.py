import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

class InferenceNode(Node):
    def __init__(self):
        super().__init__("inference_node")
        self.subscription = self.create_subscription(Image, "image_data", self.image_callback, 10)
        self.led_publisher = self.create_publisher(String, "led_command", 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to a numpy array
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Save the image as a JPEG file, overwriting the previous file
            filename = "latest_image.jpg"
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f"Image saved as {filename}")
        
        except Exception as e:
            # Publish an error message to led_command topic
            self.led_publisher.publish(String(data="error"))
            self.get_logger().error(f"Failed to process image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = InferenceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

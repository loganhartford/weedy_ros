#!/mnt/shared/weedy_ros/src/inference/inference/yolo_env/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np

from ultralytics import YOLO

class InferenceNode(Node):
    def __init__(self):
        super().__init__("inference_node")
        self.subscription = self.create_subscription(Image, "image_data", self.image_callback, 10)
        self.led_publisher = self.create_publisher(String, "led_command", 10)
        # self.bridge = CvBridge()
        self.model_path = "/home/weedy/shared/weedy_ros/src/inference/inference/models/indoor_pose_ncnn_model"
        self.model = YOLO(self.model_path, task="pose")

    def image_callback(self, msg):
        try:
           # Manually decode the ROS Image message to a numpy array
            height = msg.height
            width = msg.width
            channels = 3  # Assuming RGB or BGR
            img_data = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, channels)

            # Run inference
            results = self.model(img_data)
            if not results:
                self.get_logger().info("No detections found.")
            else:
                result = results[0]
                
                # Print detection details
                for box in result.boxes:
                    xyxy = box.xyxy[0].cpu().numpy() # This call to cpu might be causing an error
                    conf = box.conf
                    cls = box.cls
                    self.get_logger().info(f"Bounding box coordinates: {xyxy}, Confidence: {conf}, Class: {cls}")
        
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

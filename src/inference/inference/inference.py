#!/mnt/shared/weedy_ros/src/inference/inference/yolo_env/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np

from ultralytics import YOLO

from custom_msgs.msg import Keypoints, Keypoint

class InferenceNode(Node):
    def __init__(self):
        super().__init__("inference_node")
        self.subscription = self.create_subscription(Image, "image_data", self.image_callback, 10)
        self.led_publisher = self.create_publisher(String, "led_command", 10)
        self.keypoint_publisher = self.create_publisher(Keypoints, "keypoints", 10)

        self.model_path = "/mnt/shared/weedy_ros/src/inference/inference/models/indoor_pose_ncnn_model"
        self.model = YOLO(self.model_path, task="pose")

    def image_callback(self, msg):
        try:
           # Manually decode the ROS Image message to a numpy array
            height = msg.height
            width = msg.width
            channels = 3  # Assuming RGB or BGR
            img_data = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, channels)

            self.get_logger().info(f"Inferencing...")

            # Run inference
            results = self.model(img_data)
            if not results:
                self.get_logger().info("No detections found.")
            else:
                result = results[0]
                result.save()
                msg = Keypoints()
                for i, keypoint in enumerate(result.keypoints):
                    data = keypoint.data
                    kp = Keypoint()

                    kp.flower = [round(x, 3) for x in data[i][0].tolist()]
                    kp.base = [round(x, 3) for x in data[i][1].tolist()]
                    kp.upper = [round(x, 3) for x in data[i][2].tolist()]
                    kp.lower = [round(x, 3) for x in data[i][3].tolist()]
                    msg.keypoints.append(kp)

                self.keypoint_publisher.publish(msg)
                self.get_logger().info(f"Published {len(msg.keypoints)} keypoints.")
        
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

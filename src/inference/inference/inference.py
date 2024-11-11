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
        self.subscription = self.create_subscription(
            Image,
            "/image_data",
            self.image_callback, 
            10
        )

        self.keypoint_publisher = self.create_publisher(
            Keypoints,
            "/keypoints",
            10
        )

        self.model_path = "/mnt/shared/weedy_ros/src/inference/inference/models/indoor_pose_ncnn_model"
        self.model = YOLO(self.model_path, task="pose")

    def image_callback(self, msg):
        try:
           # Decode the ROS Image message to a numpy array
            height = msg.height
            width = msg.width
            channels = 3
            img_data = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, channels)

            self.get_logger().info(f"Inferencing...")

            # Run inference
            kp_msg = Keypoint()
            keypoints_msg = Keypoints()
            results = self.model(img_data)
            if not results:
                self.get_logger().info("No detections found.")
                kp_msg.has_visible = False
                keypoints_msg.keypoints.append(kp_msg)
            else:
                result = results[0]
                # result.save() # will save jpg
                
                for kp in result.keypoints:
                    data = kp.data[0]

                    kp_msg.has_visible = True
                    kp_msg.flower = [round(x, 3) for x in data[0].tolist()]
                    kp_msg.base = [round(x, 3) for x in data[1].tolist()]
                    kp_msg.upper = [round(x, 3) for x in data[2].tolist()]
                    kp_msg.lower = [round(x, 3) for x in data[3].tolist()]
                    
                    keypoints_msg.keypoints.append(kp_msg)

            self.keypoint_publisher.publish(keypoints_msg)
            self.get_logger().info(f"Published {len(keypoints_msg.keypoints)} keypoints.")
        
        except Exception as e:
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

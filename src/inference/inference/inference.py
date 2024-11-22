#!/mnt/shared/weedy_ros/src/inference/inference/yolo_env/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np

from ultralytics import YOLO

from custom_msgs.msg import Keypoint, KeypointSet, Inference

class InferenceNode(Node):
    def __init__(self):
        super().__init__("inference_node")

        self.subscription = self.create_subscription(Image, "/image_data", self.image_callback, 10)
        self.keypoint_publisher = self.create_publisher(Inference, "/keypoints", 10)

        self.model_path = "/mnt/shared/weedy_ros/src/inference/inference/models/indoor_pose_ncnn_model"
        self.model = YOLO(self.model_path, task="pose", verbose=False)
        self.confidence_threshold = 0.6

        self.null_keypoint = Keypoint()
        self.null_keypoint.x = 0
        self.null_keypoint.y = 0
        self.null_keypoint.confidence = 0

    def image_callback(self, msg):
        try:
            # Decode the ROS Image message to a numpy array
            img_data = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)

            # Initialize inference message
            inference_msg = Inference()

            # Run inference
            results = self.model(img_data, verbose=False)
            results[0].save()
            if not results:
                self.append_empty_keypoint_set(inference_msg)
            else:
                self.process_results(results[0], inference_msg)


            self.keypoint_publisher.publish(inference_msg)

            self.get_logger().info(f"Published {len(inference_msg.keypoints)} keypoint sets.")

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def append_empty_keypoint_set(self, inference_msg):
        keypoint_set_msg = KeypointSet()
        keypoint_set_msg.has_visible = False
        inference_msg.keypoints.append(keypoint_set_msg)

    def process_results(self, result, inference_msg):
        for kp in result.keypoints:
            keypoint_set_msg = KeypointSet()
            keypoint_set_msg.has_visible = kp.has_visible
            
            if not kp.has_visible:
                continue
            
            # Extract and assign each keypoint's values
            for key, data in zip(["flower", "base", "upper", "lower"], kp.data[0]):
                setattr(keypoint_set_msg, key, self.create_keypoint(data))

            # Filter keypoints based on confidence threshold
            self.filter_keypoints_by_confidence(keypoint_set_msg)

            inference_msg.keypoints.append(keypoint_set_msg)

    def create_keypoint(self, data):
        keypoint = Keypoint()
        keypoint.x = round(float(data[0]), 3)
        keypoint.y = round(float(data[1]), 3)
        keypoint.confidence = round(float(data[2]), 3)
        return keypoint

    def filter_keypoints_by_confidence(self, keypoint_set_msg):
        for key in ["flower", "base", "upper", "lower"]:
            keypoint = getattr(keypoint_set_msg, key)
            if keypoint.confidence < self.confidence_threshold:
                setattr(keypoint_set_msg, key, self.null_keypoint)


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

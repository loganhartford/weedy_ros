#!/mnt/shared/weedy_ros/src/inference/inference/yolo_env/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

import numpy as np
import cv2

from ultralytics import YOLO

from custom_msgs.msg import Keypoint, KeypointSet, Inference

class InferenceNode(Node):
    def __init__(self):
        super().__init__("inference_node")

        self.subscription = self.create_subscription(Image, "/image_data", self.image_callback, 10)
        self.keypoint_publisher = self.create_publisher(Inference, "/keypoints", 10)
        self.get_img_publisher = self.create_publisher(Bool, '/get_img', 10)

        self.model_path = "/mnt/shared/weedy_ros/src/inference/inference/models/indoor_pose_ncnn_model"
        self.model = YOLO(self.model_path, task="pose", verbose=False)
        self.confidence_threshold = 0.6

        self.null_keypoint = Keypoint()
        self.null_keypoint.x = 0.0
        self.null_keypoint.y = 0.0
        self.null_keypoint.confidence = 0.0

        # Homography calibration
        self.pixel_points = np.array([[265, 443], [258, 874], [1633, 869], [1625, 440]], dtype=np.float32)
        self.ground_points = np.array([[294, 101.4], [294, 0], [-26, 0], [-26, 101.4]], dtype=np.float32)
        self.H, _ = cv2.findHomography(self.pixel_points, self.ground_points)

    def image_callback(self, msg):
        try:
            # Decode the ROS Image message to a numpy array
            img_data = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)

            # Initialize inference message
            inference_msg = Inference()

            # Run inference
            results = self.model(img_data, verbose=False)
            
            if not results:
                # self.append_empty_keypoint_set(inference_msg)
                pass
            else:
                result = results[0]

                self.get_logger().info(f"{len(result.keypoints)}")
                self.get_logger().info(f"{len(result.boxes)}")

                # Filter by box confidence
                result.keypoints = result.keypoints[result.boxes.conf >= self.confidence_threshold]
                result.boxes = result.boxes[result.boxes.conf >= self.confidence_threshold]

                # Optionally save the image
                result.save("/mnt/shared/weedy_ros/src/inference/inference/result.jpg")

                self.process_results(results[0], inference_msg)
                self.keypoint_publisher.publish(inference_msg)
                self.get_logger().info(f"Published {len(inference_msg.keypoints)} keypoint sets.")

            self.publish_img_request()

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")
    
    def publish_img_request(self):
        msg = Bool()
        msg.data = True
        self.get_img_publisher.publish(msg)
        self.get_logger().info(f"Image requested.")

    def append_empty_keypoint_set(self, inference_msg):
        keypoint_set_msg = KeypointSet()
        keypoint_set_msg.has_visible = False
        keypoint_set_msg.base = self.null_keypoint
        keypoint_set_msg.flower = self.null_keypoint
        keypoint_set_msg.upper = self.null_keypoint
        keypoint_set_msg.lower = self.null_keypoint
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
        ground_coords = self.homography_transform(data[:2])

        keypoint = Keypoint()
        keypoint.x = round(float(ground_coords[0]), 3)
        keypoint.y = round(float(ground_coords[1]), 3)
        keypoint.confidence = round(float(data[2]), 3)
        return keypoint

    def filter_keypoints_by_confidence(self, keypoint_set_msg):
        for key in ["flower", "base", "upper", "lower"]:
            keypoint = getattr(keypoint_set_msg, key)
            if keypoint.confidence < self.confidence_threshold:
                setattr(keypoint_set_msg, key, self.null_keypoint)
    
    def homography_transform(self, pixel):
        image_point = np.array([pixel[0], pixel[1], 1], dtype=np.float32)
        ground_point = np.dot(self.H, image_point)
        ground_point /= ground_point[2]

        return ground_point[:2]

    def destroy_node(self):
        super().destroy_node()


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

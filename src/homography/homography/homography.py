import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

from custom_msgs.msg import Points, Inference

import cv2

class HomographyNode(Node):
    def __init__(self):
        super().__init__("homography_node")

        self.subscriber = self.create_subscription(Inference, "/keypoints", self.keypoints_callback, 10)
        self.publisher = self.create_publisher(Points, "/cartesian_coordinates", 10)

        self.confidence_threshold = 0.8

    def keypoints_callback(self, inference_msg):
        points_msg = Points()
        for keypoint_set in inference_msg.keypoints:
            if keypoint_set.has_visible:
                # Take the most valid keypoint from each set of keypoints
                if keypoint_set.base.confidence > self.confidence_threshold:
                    # Show the homography point for debugging
                    image = cv2.imread("/mnt/shared/weedy_ros/src/inference/inference/result.jpg")
                    pixel_location = (int(keypoint_set.base.x), int(keypoint_set.base.y))
                    print(pixel_location)
                    cv2.circle(image, pixel_location, radius=10, color=(0, 255, 0), thickness=-1)
                    cv2.imwrite("./homography.jpg", image)
                    points_msg.points.append(self.convert_to_cartesian(keypoint_set.base))
                elif keypoint_set.flower.confidence > self.confidence_threshold:
                    points_msg.points.append(self.convert_to_cartesian(keypoint_set.flower))
                elif keypoint_set.lower.confidence > self.confidence_threshold:
                    points_msg.points.append(self.convert_to_cartesian(keypoint_set.lower))
                elif keypoint_set.upper.confidence > self.confidence_threshold:
                    points_msg.points.append(self.convert_to_cartesian(keypoint_set.upper))
        
        self.get_logger().info(f"Published {len(points_msg.points)} cartesian coordinates.")
        self.publisher.publish(points_msg)
    
    def map_value(self, x, input_min, input_max, output_min, output_max):
        center_input = (input_max + input_min) / 2  # Center of input range
        center_output = (output_max + output_min) / 2  # Center of output range
        return (x - center_input) / (input_max - input_min) * (output_max - output_min) + center_output

    def convert_to_cartesian(self, keypoint):
        point_msg = Point()

        # Define input dimensions (image size in pixels)
        image_width = 1920
        image_height = 1080

        # Define output dimensions (cartesian range, e.g., robot frame)
        output_x_range = (-225, 225)  # Assuming 450 total width
        output_y_range = (-225, 225)  # Assuming 450 total height

        # Map keypoint coordinates relative to the center of the image
        point_msg.y = self.map_value(keypoint.x, 0, image_width, output_x_range[0], output_x_range[1])
        point_msg.x = self.map_value(keypoint.y, 0, image_height, output_y_range[0], output_y_range[1])
        point_msg.z = 0.0  # Assuming Z is 0 for now

        return point_msg


def main(args=None):
    rclpy.init(args=args)
    node = HomographyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

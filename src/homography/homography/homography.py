import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

from custom_msgs.msg import Points

class HomographyNode(Node):
    def __init__(self):
        super().__init__("homography_node")

        self.subscriber = self.create_subscription(Point, "/keypoints", self.keypoints_callback, 10)
        self.publisher = self.create_publisher(Points, "/cartesian_coordinates", 10)

        self.confidence_threshold = 0.8

    def keypoints_callback(self, msg):
        points_msg = Points()
        for keypoint_set in msg:
            if keypoint_set.has_visible:
                # Take the most valid keypoint from each set of keypoints
                if keypoint_set.base.confidence > self.confidence_threshold:
                    points_msg.append(self.convert_to_cartesian(keypoint_set.base))
                elif keypoint_set.flower.confidence > self.confidence_threshold:
                    points_msg.append(self.convert_to_cartesian(keypoint_set.flower))
                elif keypoint_set.lower.confidence > self.confidence_threshold:
                    points_msg.append(self.convert_to_cartesian(keypoint_set.lower))
                elif keypoint_set.upper.confidence > self.confidence_threshold:
                    points_msg.append(self.convert_to_cartesian(keypoint_set.upper))
        
        self.get_logger().info(f"Published {len(points_msg)} cartesian coordinates.")
        self.publisher.publish(points_msg)
    
    def map_value(self, x, input_min=0, input_max=1920, output_min=0, output_max=450):
        return (x - input_min) / (input_max - input_min) * (output_max - output_min) + output_min

    def convert_to_cartesian(self, keypoint):
        point_msg = Point()
        # Dummy code, acutally need to convert pixel coordinates to cartesian
        # and then convert to robot frame.
        point_msg.x = self.map_value(keypoint.x, input_max=1920)
        point_msg.y = self.map_value(keypoint.y, input_max=1080)
        point_msg.z = 0.0
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

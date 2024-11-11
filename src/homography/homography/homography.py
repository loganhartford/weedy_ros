import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

from custom_msgs import Points

class HomographyNode(Node):
    def __init__(self):
        super().__init__("homography_node")

        self.subscriber = self.create_subscription(
            Point,
            "/keypoints",
            self.keypoints_callback,
            10
        )

        self.publisher = self.create_publisher(
            Points,
            "/cartesian_coordinates",
            10
        )

        self.confidence_threshold = 0.8

    def keypoints_callback(self, msg):
        points_msg = Points()
        for keypoint_msg in msg:
            if not keypoint_msg.has_visible:
                continue
            # Take the first most valid keypoint from each set of keypoints
            elif keypoint_msg.base[2] > self.confidence_threshold:
                points_msg.append(self.convert_to_cartesian(keypoint_msg.base))
            elif keypoint_msg.flower[2] > self.confidence_threshold:
                points_msg.append(self.convert_to_cartesian(keypoint_msg.flower))
            elif keypoint_msg.lower[2] > self.confidence_threshold:
                points_msg.append(self.convert_to_cartesian(keypoint_msg.lower))
            elif keypoint_msg.upper[2] > self.confidence_threshold:
                points_msg.append(self.convert_to_cartesian(keypoint_msg.upper))
        
        if len(points_msg) > 0:
            self.get_logger().info(f"Published {len(points_msg)} keypoint coordinates.")
            self.publisher.publish(points_msg)
    
    def map_value(self, x, input_min=0, input_max=1920, output_min=0, output_max=450):
        return (x - input_min) / (input_max - input_min) * (output_max - output_min) + output_min

    def convert_to_cartesian(self, keypoint_data):
        point_msg = Point()
        point_msg.x = self.map_value(keypoint_data[0], input_max=1920)
        point_msg.y = self.map_value(keypoint_data[1], input_max=1080)
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

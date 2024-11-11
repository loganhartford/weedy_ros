import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class HomographyNode(Node):
    def __init__(self):
        super().__init__("homography_node")

        self.subscriber = self.create_subscription(
            Point,
            "/pixel_coordinates",
            self.pixel_coordinates_callback,
            10
        )

        self.publisher = self.create_publisher(
            Point,
            "/cartesian_coordinates",
            10
        )

    def pixel_coordinates_callback(self, msg):
        outPoint = Point()
        outPoint.x = self.map_value(msg.x, input_max=1920)
        outPoint.y = self.map_value(msg.y, input_max=1080)
        self.publisher.publish(outPoint)
        self.get_logger().info(f"Published cartesian coordinate: {outPoint}.")
    
    def map_value(self, x, input_min=0, input_max=1920, output_min=0, output_max=450):
        return (x - input_min) / (input_max - input_min) * (output_max - output_min) + output_min


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

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool, String

from custom_msgs.msg import Keypoints, Keypoint

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Subscribers
        self.cmd_subscription = self.create_subscription(
            String,
            '/cmd',
            self.cmd_callback,
            10
        )
        self.keypoints_subscription = self.create_subscription(
            Keypoints,
            '/keypoints',
            self.keypoints_callback,
            10
        )
        # self.cartesian_state_subscription = self.create_subscription(
        #     CartesianState,
        #     '/cartesian_state',
        #     self.cartesian_state_callback,
        #     10
        # )
        self.cartesian_coordinates_subscription = self.create_subscription(
            Point,
            '/cartesian_coordinates',
            self.cartesian_coordinates_callback,
            10
        )

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        # self.cmd_cartesian_publisher = self.create_publisher(
        #     CartesianState,
        #     '/cmd_cartesian',
        #     10
        # )
        self.get_img_publisher = self.create_publisher(
            Bool,
            '/get_img',
            10
        )
        self.pixel_coordinates_publisher = self.create_publisher(
            Point,
            '/pixel_coordinates',
            10
        )

    def cmd_callback(self, msg):
        self.get_logger().info(f"Received cmd: {msg.data}")
        if msg.data == "start":
            self.get_logger().info(f"Starting...")
        elif msg.data == "get_img":
            self.publish_img_request()
        elif msg.data == "test_uart":
            self.get_logger().info(f"Sending test msg...")

    def keypoints_callback(self, msg):
        self.get_logger().info(f"Received keypoints: {msg}")
        for keypoint in msg:
            # More complicated logic later
            if keypoint.base[2] > 0.7: # Confidence threshold
                point = Point()
                point.x = keypoint.base[0]
                point.y = keypoint.base[1]
                point.z = 0.0
                self.publish_pixel_coordinates(point)



    def cartesian_state_callback(self, msg):
        self.get_logger().info(f"Received cartesian state: {msg}")

    def cartesian_coordinates_callback(self, msg):
        self.get_logger().info(f"Received cartesian coordinates: {msg}")      

    def publish_img_request(self):
        msg = Bool()
        msg.data = True
        self.get_img_publisher.publish(msg)
        self.get_logger().info(f"Image requested.")
    
    def publish_pixel_coordinates(self, point):
        self.pixel_coordinates_publisher.publish(point)
        self.get_logger.info(f"Posted pixel coordinates: {point}")



def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
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
        # self.cartesian_coordinates_subscription = self.create_subscription(
        #     CartesianCoordinates,
        #     '/cartesian_coordinates',
        #     self.cartesian_coordinates_callback,
        #     10
        # )

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

    def cmd_callback(self, msg):
        self.get_logger().info(f"Received cmd: {msg.data}")

    def keypoints_callback(self, msg):
        self.get_logger().info(f"Received keypoints: {msg}")

    def cartesian_state_callback(self, msg):
        self.get_logger().info(f"Received cartesian state: {msg}")

    def cartesian_coordinates_callback(self, msg):
        self.get_logger().info(f"Received cartesian coordinates: {msg}")

    def publish_img_request(self):
        msg = Bool()
        msg.data = True
        self.get_img_publisher.publish(msg)
        self.get_logger().info(f"Image requested.")

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

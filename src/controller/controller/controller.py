import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool, String

from custom_msgs.msg import Keypoints, Keypoint, CartesianMsg

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
        self.cartesian_state_subscription = self.create_subscription(
            CartesianMsg,
            '/cartesian_state',
            self.cartesian_state_callback,
            10
        )
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
        self.cmd_cartesian_publisher = self.create_publisher(
            CartesianMsg,
            '/cmd_cartesian',
            10
        )
        self.get_img_publisher = self.create_publisher(
            Bool,
            '/get_img',
            10
        )

        # Axis encodings
        self.x_axis = 0 
        self.y_axis = 1
        self.z_axis = 2

        # State
        self.state = "idle"
        self.positions = [0,0,0]
        self.velocities = [0, 0] # linear, angular

        # Thresholds
        self.search_confidence = 0.6
        self.locating_confidence = 0.8

    def cmd_callback(self, msg):
        self.get_logger().info(f"Received cmd: {msg.data}")
        if msg.data == "start":
            self.get_logger().info(f"Starting...")
            self.state = "searching"

            self.publish_img_request()
            self.publish_twist(0.5, 0)
            
        elif msg.data == "get_img":
            self.publish_img_request()
        
        elif msg.data == "test_uart":
            self.get_logger().info(f"Sending test msg...")
            testMsg = CartesianMsg()
            testMsg.axis = 1
            testMsg.position = 258
            self.cmd_cartesian_publisher.publish(testMsg)
        
        elif msg.data == "stop":
            self.get_logger().info(f"Stopping...")
            self.state = "idle"

    def keypoints_callback(self, msg):
        self.get_logger().info(f"Received keypoints: {msg}")
        

    def cartesian_state_callback(self, msg):
        self.get_logger().info(f"Received cartesian state: {msg}")

    def cartesian_coordinates_callback(self, msg):
        self.get_logger().info(f"Received cartesian coordinates: {msg}")
        if msg.x:
            # send command to locomotion
            pass
        if msg.y >0 and msg.y <= 450:
            outMsg = CartesianMsg()
            outMsg.axis = self.y_axis
            outMsg.position = msg.y
            self.cmd_cartesian_publisher.publish(outMsg)

    def publish_img_request(self):
        msg = Bool()
        msg.data = True
        self.get_img_publisher.publish(msg)
        self.get_logger().info(f"Image requested.")
    
    def publish_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.anglular.z = angular
        self.cmd_vel_publisher.publish(msg)
        self.velocities = [linear, angular]
        self.get_logger().info(f"Setting linear: {linear}, angular: {angular}")



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

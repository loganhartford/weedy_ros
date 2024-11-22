import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool, String

from custom_msgs.msg import Keypoint, KeypointSet, Inference, CartesianCmd, Points

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Subscribers
        self.cmd_subscription = self.create_subscription(String, '/cmd', self.cmd_callback, 10)
        self.keypoints_subscription = self.create_subscription(Inference, '/keypoints', self.keypoints_callback, 10)
        self.cartesian_state_subscription = self.create_subscription(CartesianCmd, '/cartesian_state', self.cartesian_state_callback, 10)
        self.cartesian_coordinates_subscription = self.create_subscription(Points, '/cartesian_coordinates', self.cartesian_coordinates_callback, 10)

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_cartesian_publisher = self.create_publisher(CartesianCmd, '/cmd_cartesian', 10)
        self.get_img_publisher = self.create_publisher(Bool, '/get_img', 10)

        # Axis encodings
        self.x_axis = 0 
        self.y_axis = 1
        self.z_axis = 2

        # Logical parameters
        self.y_axis_alignment_tolerance = 5 # mm

        # State
        self.state = "idle"
        self.empty_homogoraphy_count = 0
    
    def transition_to_state(self, state):
        if self.state == state:
            self.get_logger().error(f"State already {state}")
            return

        self.get_logger().info(f"Transitioning from {self.state} to {state}.")

        if self.state == "idle":
            if state == "searching":
                self.state = "searching"
                self.publish_img_request()
                self.publish_twist(0.5, 0)
        elif self.state == "searching":
            if state == "idle":
                self.state = "idle"
                self.publish_twist(0, 0)
            elif state == "finding":
                self.state = "finding"
                self.publish_img_request()
                self.publish_twist(0, 0)
        elif self.state == "finding":
            if state == "idle":
                self.state = "idle"
            elif state == "waiting":
                self.state = "waiting"
        elif self.state == "waiting":
            if state == "idle":
                self.state = "idle"
            elif state == "searching":
                self.state = "searching"
                self.publish_img_request()
                self.publish_twist(0.5, 0)
        else:
            self.get_logger().error(f"Transitioning from {self.state} to {state} is invalid.")
            return
            
        

    def cmd_callback(self, msg):
        if msg.data == "start":
            self.transition_to_state("searching")
        elif msg.data == "stop":
            self.transition_to_state("idle")
        # For dev
        elif msg.data == "get_img":
            self.publish_img_request()
        elif msg.data == "test_uart":
            testMsg = CartesianCmd()
            testMsg.axis = 1
            testMsg.position = 258
            self.cmd_cartesian_publisher.publish(testMsg)
        else:
            self.get_logger().error(f"'{msg}' is not a valid command.")
        
    def keypoints_callback(self, msg):
        for keypoint_set in msg.keypoints:
            # Want to stop the robot if a valid keypoint is posted
            if self.state == "searching":
                if keypoint_set.has_visible:
                    self.transition_to_state("finding")
                    return
        # No keypoints detected, request another image
        if self.state == "searching":
            self.publish_img_request()

    def cartesian_coordinates_callback(self, msg):
        if self.state == "finding":
            # Homography located a valid keypoint
            if len(msg.points) > 0:
                # Sort by alignment to y-axis
                best_point = min(msg.points, key=lambda point: point.x)

                # Once aligned on y axis
                if abs(best_point.x) < self.y_axis_alignment_tolerance:
                    # Send command to the Nucleo
                    cartesian_msg = CartesianCmd()
                    cartesian_msg.axis = self.y_axis
                    cartesian_msg.position = best_point.y
                    self.cmd_cartesian_publisher.publish(cartesian_msg)
                    self.transition_to_state("waiting")
                # If we are not aligned on y axis, move in x
                else:
                    self.get_logger().info(f"Moving {best_point.x} mm in x")
                    # probably transition to waiting and wait for a callback but not yet
                    self.publish_img_request()

                self.empty_homogoraphy_count = 0
            # Homography should have located a keypoints
            else:
                self.empty_homogoraphy_count += 1
                # Possibly a false positive initially, move on
                if self.empty_homogoraphy_count >=3:
                    self.transition_to_state("searching")
                else:
                    self.publish_img_request()

    def cartesian_state_callback(self, msg):
        if msg.axis == -1:
            self.transition_to_state("idle")
        else:
            self.transition_to_state("searching")
        

    def publish_img_request(self):
        msg = Bool()
        msg.data = True
        self.get_img_publisher.publish(msg)
        self.get_logger().info(f"Image requested.")
    
    def publish_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
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

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool, String

import time

from custom_msgs.msg import Keypoint, KeypointSet, Inference, CartesianCmd, Points

class DecisionsNode(Node):
    def __init__(self):
        super().__init__('decisions_node')

        # Subscribers
        self.cmd_subscription = self.create_subscription(String, '/cmd', self.cmd_callback, 10)
        self.keypoints_subscription = self.create_subscription(Inference, '/keypoints', self.keypoints_callback, 10)
        self.cartesian_state_subscription = self.create_subscription(CartesianCmd, '/cartesian_state', self.cartesian_state_callback, 10)

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
        self.no_points_count = 0
        self.no_points_threshold = 3

        # Dandelion detection
        self.confidence_threshold = 0.8
    
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
            if state == "test":
                self.state = "test"
        elif self.state == "searching":
            if state == "idle":
                self.state = "idle"
                self.publish_twist(0, 0)
            elif state == "finding":
                self.state = "finding"
                # self.publish_img_request()
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
                # self.publish_img_request()
                self.publish_twist(0.5, 0)
        else:
            self.get_logger().error(f"Transitioning from {self.state} to {state} is invalid.")
            return

    def cmd_callback(self, msg):
        if msg.data == "start":
            self.transition_to_state("searching")
        elif msg.data == "stop":
            self.transition_to_state("idle")
        elif msg.data == "test":
            self.transition_to_state("test")
        # For dev
        elif msg.data == "get_img":
            self.publish_img_request()
        elif msg.data == "test_uart":
            testMsg = CartesianCmd()
            testMsg.axis = 1
            testMsg.position = 100
            self.cmd_cartesian_publisher.publish(testMsg)
        else:
            self.get_logger().error(f"'{msg}' is not a valid command.")
        
    def keypoints_callback(self, msg):
        if self.state != "searching" and self.state != "finding" and self.state != "test":
            return

        keypoints = []
        for keypoint_set in msg.keypoints:
            if not keypoint_set.has_visible:
                continue
            # Want to stop the robot if a valid keypoint is posted
            if self.state == "searching":
                self.transition_to_state("finding")
            
            # Take the most valid keypoint from each set of keypoints
            if keypoint_set.base.confidence > self.confidence_threshold:
                keypoints.append(keypoint_set.base)
            elif keypoint_set.flower.confidence > self.confidence_threshold:
                keypoints.append(keypoint_set.flower)
            elif keypoint_set.lower.confidence > self.confidence_threshold:
                keypoints.append(keypoint_set.lower)
            elif keypoint_set.upper.confidence > self.confidence_threshold:
                keypoints.append(keypoint_set.upper)
        
        self.process_points(keypoints)

    def process_points(self, points):
        if self.state != "finding" and self.state != "test":
            return
        
        if len(points) > 0:
            self.no_points_count = 0
            # Take the point closest to the y axis
            best_point = min(points, key=lambda point: abs(point.x))
            self.get_logger().info(f"{best_point.x}, {best_point.y}")

            # Once aligned on y axis, send command to the Nucleo
            if abs(best_point.x) < self.y_axis_alignment_tolerance:
                cartesian_msg = CartesianCmd()
                cartesian_msg.axis = self.y_axis
                cartesian_msg.position = int(abs(best_point.y))
                self.cmd_cartesian_publisher.publish(cartesian_msg)
                self.transition_to_state("waiting")
            # If we are not aligned on y axis, move in x
            else:
                self.get_logger().info(f"Moving {best_point.x} mm in x")
                # probably transition to waiting and wait for a callback but not yet
        else:
            self.no_points_count += 1
            # Possibly a false positive initially, move on
            if self.no_points_count >= self.no_points_threshold:
                self.transition_to_state("searching")
            # else:
            #     self.publish_img_request()

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
    node = DecisionsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

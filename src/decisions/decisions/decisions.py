import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import numpy as np
import cv2
from enum import Enum, auto

from utils.uart import UART
from utils.neopixel_ring import NeoPixelRing
from decisions.yolo_model import YOLOModel

class State(Enum):
    IDLE = auto()
    EXPLORING = auto()
    ALIGNING = auto()
    WAITING = auto()

class DecisionsNode(Node):
    def __init__(self):
        super().__init__('decisions_node')

        # Subscribers
        self.cmd_subscription = self.create_subscription(String, '/cmd', self.cmd_callback, 10)

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.uart = UART()
        self.y_axis_alignment_tolerance = 5 # mm

        # State
        self.state = "idle"
        self.no_points_count = 0
        self.no_points_threshold = 3

        # Computer Vision
        self.cv = YOLOModel()
        self.conf_thresh = 0.8
        self.led_ring = NeoPixelRing()

        # Homography
        self.pixel_points = np.array([
            [265, 443], [258, 874], [1633, 869], [1625, 440]
                                                ], dtype=np.float32)
        self.ground_points = np.array([
            [101.4, -48.33], [0, -48.33], [0, 271.67], [101.4, 271.67]
                                                ], dtype=np.float32)
        self.H, _ = cv2.findHomography(self.pixel_points, self.ground_points)

        """ To use later:
        # Filter by box confidence
        
        """

        # Valid state transitions
        self.valid_transitions = {
            State.IDLE: [State.EXPLORING],
            State.EXPLORING: [State.IDLE, State.ALIGNING],
            State.ALIGNING: [State.IDLE, State.WAITING],
            State.WAITING: [State.IDLE, State.EXPLORING],
        }

        self.get_logger().info(f"Decisions Init")

    def serach(self):
        # Look for dandelions
        self.publish_twist(0.3, 0)

        while self.state == State.EXPLORING:
            result = self.cv.run_inference()

            if result == None:
                continue
            
            # Filter result by box confidence and keypoints visibility
            valid_indices = (result.boxes.conf >= self.conf_thresh) & np.array([kp.has_visible for kp in result.keypoints])

            result.keypoints = result.keypoints[valid_indices]
            result.boxes = result.boxes[valid_indices]


            if len(result.keypoints) == 0:
                continue
            
            kp_list = []
            # TODO: Validate
            """
            Idea:
                - For each set of keypoints, take the closest to the ground if it showed up
                - If there are multiple inference results, there could be multiple points in kp_list
                - Taking the most confident one is not necisarrily best
                - Also need to handle case where two dandelions are in the frame
                    - Alwasy just take the closer one.
                    - as you move closer to it, it will be closer.
            """
            for kp in result.keypoints:
                keypoints = list(zip(["flower", "base", "upper", "lower"], kp.data[0]))
                keypoints = sorted(keypoints, key=lambda x: ["base", "lower", "upper", "flower"].index(x[0]))
                for name, tensor in keypoints:
                    kp_conf = tensor[2]
                    if tensor[2] != 0.0:
                        kp_list.append(tensor)
                        break





    # TODO: nuke
    def keypoints_callback(self, msg):
        if self.state != "EXPLORING" and self.state != "ALIGNING":
            return

        keypoints = []
        for keypoint_set in msg.keypoints:
            if not keypoint_set.has_visible:
                continue
            # Want to stop the robot if a valid keypoint is posted
            if self.state == "EXPLORING":
                self.transition_to_state("ALIGNING")
            
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
        if self.state != "ALIGNING":
            return
        
        if len(points) > 0:
            self.no_points_count = 0
            # Take the point closest to the y axis
            best_point = min(points, key=lambda point: abs(point.x))
            self.get_logger().info(f"{best_point.x}, {best_point.y}")

            # Once aligned on y axis, send command to the Nucleo
            # TODO: Update "waiting" logic
            if abs(best_point.x) < self.y_axis_alignment_tolerance:
                self.uart.send_command(1, int(abs(best_point.y)))
                self.transition_to_state("waiting")
            # If we are not aligned on y axis, move in x
            else:
                self.get_logger().info(f"Moving {best_point.x} mm in x")
                # probably transition to waiting and wait for a callback but not yet
        else:
            self.no_points_count += 1
            # Possibly a false positive initially, move on
            if self.no_points_count >= self.no_points_threshold:
                self.transition_to_state("EXPLORING")
            
        
    def homography_transform(self, pixel):
        image_point = np.array([pixel[0], pixel[1], 1], dtype=np.float32)
        ground_point = np.dot(self.H, image_point)
        ground_point /= ground_point[2]

        return ground_point[:2]
    
    def publish_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_publisher.publish(msg)
        self.velocities = [linear, angular]
        self.get_logger().info(f"Setting linear: {linear}, angular: {angular}")

    def cmd_callback(self, msg):
        command_map = {
            "start": State.EXPLORING,
            "stop": State.IDLE,
        }

        if msg.data in command_map:
            self.transition_to_state(command_map[msg.data])
        elif msg.data == "get_img":
            self.cv.capture_and_save_image()
        else:
            self.get_logger().error(f"'{msg}' is not a valid command.")

    def transition_to_state(self, new_state):
        if new_state == self.state:
            self.get_logger().error(f"Already in state {self.state.name}")
            return

        if new_state not in self.valid_transitions.get(self.state, []):
            self.get_logger().error(f"Invalid transition from {self.state.name} to {new_state.name}")
            return

        self.get_logger().info(f"Transitioning from {self.state.name} to {new_state.name}")
        self.state = new_state
        self.on_state_entry()
    
    def on_state_entry(self):
        if self.state == State.EXPLORING:
            self.search()
        elif self.state == State.IDLE:
            self.publish_twist(0, 0)
        elif self.state == State.ALIGNING:
            self.publish_twist(0, 0)
        elif self.state == State.WAITING:
            pass

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DecisionsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()

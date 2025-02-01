import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import numpy as np
import cv2
from enum import Enum, auto
import time

from utils.uart import UART
from utils.neopixel_ring import NeoPixelRing
from decisions.yolo_model import YOLOModel
from utils.robot_params import y_axis_max

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
        self.odom_subscription = self.create_subscription(Odometry, "/odom", self.odom_callback, 1)

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_pose_publisher = self.create_publisher(PoseStamped, '/cmd_pose', 10)

        self.uart = UART()
        self.y_axis_alignment_tolerance = 5 # mm
        self.y_axis_max = y_axis_max #mm

        # State
        self.state = State.IDLE
        self.no_points_count = 0
        self.no_points_threshold = 3
        self.move_timeout = 5 # s

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

        # Valid state transitions
        self.valid_transitions = {
            State.IDLE: [State.EXPLORING],
            State.EXPLORING: [State.IDLE, State.ALIGNING],
            State.ALIGNING: [State.IDLE, State.WAITING],
            State.WAITING: [State.IDLE, State.EXPLORING],
        }

        self.led_ring.set_color(255, 255, 255, 0.5)

        self.get_logger().info(f"Decisions Init")

    def explore(self):
        # Drive slow and look for dandelions
        self.publish_twist(0.3, 0)

        while self.state == State.EXPLORING:
            if self.get_keypoints():
                self.transition_to_state(State.ALIGNING)
                
    def align(self):
        # Stop
        self.publish_twist(0, 0)

        self.no_points_count = 0
        self.no_points_threshold  = 3
        while self.state == State.ALIGNING:
            kp_list = self.get_keypoints()
            if kp_list == None:
                self.no_points_count += 1
                if self.no_points_count >= self.no_points_threshold :
                    self.transition_to_state(State.EXPLORING)
                continue
            
            # Got some keypoints
            self.no_points_count = 0

            # Take point closest to the y-axis
            best_point = min(kp_list, key=lambda point: abs(point[0]))

            # y-axis is aligned
            if abs(best_point[0]) < self.y_axis_alignment_tolerance:
                # Trigger the removal
                self.uart.send_command(1, best_point[1])
                self.transition_to_state(State.WAITING)
                continue

            # Add the displacement from the y-axis
            pose_msg = self.pose
            pose_msg.pose_msg.pose.pose.position.x += best_point[0] / 1000 # convert to m

            # Move the robot
            self.transition_to_state(State.WAITING)
            self.cmd_pose_publisher.publish(pose_msg)

            # Wait for the move to finish (update in cmd_callback)
            start_time = time.time()
            elapsed_time = 0
            while(self.state == State.WAITING):
                elapsed_time = time.time() - start_time
                if elapsed_time > self.move_timeout:
                    self.get_logger().error(f"Timeout waiting to move.")
                    self.transition_to_state(State.IDLE)
                time.sleep(0.1)

    def get_keypoints(self, keypoints):
        result = self.cv.run_inference()

        if result == None:
            return None
        
        # Filter result by box confidence and keypoints visibility
        valid_indices = (result.boxes.conf >= self.conf_thresh) & np.array([kp.has_visible for kp in result.keypoints])
        result.keypoints = result.keypoints[valid_indices]
        result.boxes = result.boxes[valid_indices]

        if len(result.keypoints) == 0:
            return None

        kp_list = []
        for kp in keypoints:
            points = list(zip(["flower", "base", "upper", "lower"], kp.data[0]))
            points = sorted(points, key=lambda x: ["base", "lower", "upper", "flower"].index(x[0]))
            """
            - Here we could choose to fileter the keypoints by confidence,
              but we've already filteres by box conf and visibility, so
              we just take the lowest visible keypoint.
            - This keypoint is fully valid as far as we are concerned
            - Filtering keypoints by conf as well may be necessary if 
              if low conf keypionts are causing issues.
            """
            # For each result, take the lowest point
            for name, tensor in points:
                kp_conf = tensor[2]
                # Keypoint is non-zero and is within the y-axis
                if tensor[2] != 0.0 and tensor[1] >= 0 and tensor[1] < self.y_axis_max:
                    kp_list.append(tensor)
                    break
        if kp_list == []:
            return None
        
        return kp_list
            
    def homography_transform(self, pixel):
        image_point = np.array([pixel[0], pixel[1], 1], dtype=np.float32)
        ground_point = np.dot(self.H, image_point)
        ground_point /= ground_point[2]

        return ground_point[:2]

    def cmd_callback(self, msg):
        command_map = {
            "start": State.EXPLORING,
            "stop": State.IDLE,
            "goal_reached": State.EXPLORING,
        }

        if msg.data in command_map:
            self.transition_to_state(command_map[msg.data])
        elif msg.data == "get_img":
            self.cv.capture_and_save_image()
        else:
            self.get_logger().error(f"'{msg}' is not a valid command.")
    
    def odom_callback(self, msg):
        self.pose = msg
    
    def publish_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_publisher.publish(msg)
        self.velocities = [linear, angular]
        self.get_logger().info(f"Setting linear: {linear}, angular: {angular}")

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
            self.explore()
        elif self.state == State.IDLE:
            self.publish_twist(0, 0)
        elif self.state == State.ALIGNING:
            self.align()
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

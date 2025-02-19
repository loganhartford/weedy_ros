import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import numpy as np
import cv2
from enum import Enum, auto

from utils.uart import UART
from utils.nucleo_gpio import NucleoGpio
from utils.neopixel_ring import NeoPixelRing
from decisions.yolo_model import YOLOModel
from utils.robot_params import y_axis_max, y_axis_alignment_tolerance, pixel_points, ground_points, explore_linear_speed

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
        self.cmd_odom_publisher = self.create_publisher(Odometry, '/cmd_odom', 10)

        # Hardware / Utils
        self.uart = UART()
        self.led_ring = NeoPixelRing()
        self.cv = YOLOModel()
        self.conf_thresh = 0.6
        self.nuc_gpio = NucleoGpio()

        # Robot parameters
        self.y_axis_alignment_tolerance = y_axis_alignment_tolerance
        self.y_axis_max = y_axis_max
        self.move_timeout = 5  # seconds
        self.battery_voltage = 37.0 # volts

        # State and timers
        self.state = State.IDLE
        self.explore_timer = None
        self.align_timer = None
        self.wait_timer = None

        # Odometry (updated in callback)
        self.odom = None

        # Compute homography matrix
        self.H, status = cv2.findHomography(pixel_points, ground_points, cv2.RANSAC, 5.0)
        if self.H is not None:
            self.get_logger().info(f"Homography Status: {status.ravel()}")
        else:
            self.get_logger().error("Homography computation failed.")

        # Valid state transitions
        self.valid_transitions = {
            State.IDLE: [State.EXPLORING],
            State.EXPLORING: [State.IDLE, State.ALIGNING],
            State.ALIGNING: [State.IDLE, State.WAITING],
            State.WAITING: [State.IDLE, State.EXPLORING, State.ALIGNING],
        }

        # Flash battery status on startup
        self.update_battery()

        # Set LED to white
        self.led_ring.set_color(255, 255, 255, 1.0)
        self.get_logger().info("Decisions Node Initialized")

    # -------------------------
    # STATE: EXPLORING
    # -------------------------
    def start_exploring(self):
        self.get_logger().info("Starting exploration.")
        # Start driving forward slowly
        self.publish_twist(explore_linear_speed, 0)
        # Create a timer to check for boxes every 0.1 sec
        if self.explore_timer is not None:
            self.explore_timer.cancel()
        self.explore_timer = self.create_timer(0.1, self.explore_callback)

    def explore_callback(self):
        if self.state != State.EXPLORING:
            if self.explore_timer:
                self.explore_timer.cancel()
            return

        boxes = self.get_boxes(save=True)
        if boxes is not None:
            self.get_logger().info("Boxes detected; transitioning to ALIGNING.")
            self.explore_timer.cancel()
            self.transition_to_state(State.ALIGNING)

    # -------------------------
    # STATE: ALIGNING
    # -------------------------
    def start_aligning(self):
        self.get_logger().info("Starting alignment.")
        # Stop the robot before aligning
        self.publish_twist(0, 0)
        # Create a timer to check for keypoints every 0.1 sec
        if self.align_timer is not None:
            self.align_timer.cancel()
        self.align_timer = self.create_timer(0.1, self.align_callback)

    def align_callback(self):
        if self.state != State.ALIGNING:
            if self.align_timer:
                self.align_timer.cancel()
            return

        self.get_logger().info("Aligning...")
        kp_list = self.get_keypoints()
        if kp_list is None:
            return  # No valid keypoints detected yet

        # Choose the keypoint closest to the y-axis (x close to 0)
        best_point = min(kp_list, key=lambda point: abs(point[0]))
        self.get_logger().info(f"Best keypoint: {best_point}")

        if abs(best_point[0]/1000.0) < self.y_axis_alignment_tolerance:
            self.get_logger().info("Y-axis aligned. Removing flower.")
            self.uart.send_command(1, best_point[1])
            self.align_timer.cancel()
            self.transition_to_state(State.WAITING)
            return

        # Otherwise, adjust the robot’s position based on the displacement.
        if self.odom is None:
            self.get_logger().warn("Odometry not yet received.")
            return

        new_odom = Odometry()
        new_odom.header = self.odom.header
        new_odom.child_frame_id = self.odom.child_frame_id
        # Adjust x by converting displacement from mm to m
        new_odom.pose.pose.position.x = self.odom.pose.pose.position.x + best_point[0] / 1000.0
        new_odom.pose.pose.position.y = self.odom.pose.pose.position.y  # Adjust y if needed
        self.get_logger().info(f"Adjusting position to x: {new_odom.pose.pose.position.x}")
        self.cmd_odom_publisher.publish(new_odom)

        self.align_timer.cancel()
        self.transition_to_state(State.WAITING)

    # -------------------------
    # STATE: WAITING
    # -------------------------
    def start_waiting(self):
        self.get_logger().info("Waiting for movement to complete.")
        self.wait_start_time = self.get_clock().now()
        if self.wait_timer is not None:
            self.wait_timer.cancel()
        self.wait_timer = self.create_timer(0.1, self.wait_callback)

    def wait_callback(self):
        if self.state != State.WAITING:
            if self.wait_timer:
                self.wait_timer.cancel()
            return

        elapsed_time = (self.get_clock().now() - self.wait_start_time).nanoseconds / 1e9
        if elapsed_time > self.move_timeout:
            self.get_logger().error("Timeout waiting to move.")
            self.wait_timer.cancel()
            self.transition_to_state(State.IDLE)
        # Otherwise, simply wait. (A "goal_reached" command via cmd_callback can change the state.)

    # -------------------------
    # UTILITY FUNCTIONS
    # -------------------------
    def get_boxes(self, save=False):
        result = self.cv.run_inference(save=save)
        if result is None:
            return None

        # Filter result by box confidence
        valid_indices = result.boxes.conf >= 0.5
        result.boxes = result.boxes[valid_indices]

        if len(result.boxes) == 0:
            return None

        return result.boxes

    def get_keypoints(self, save=False):
        result = self.cv.run_inference(save=save)
        if result is None:
            return None

        if len(result.keypoints) == 0:
            return None

        kp_list = []
        for kp in result.keypoints:
            # Zip names with points and sort by desired order
            points = list(zip(["flower", "base", "upper", "lower"], kp.data[0]))
            points = sorted(points, key=lambda x: ["base", "lower", "upper", "flower"].index(x[0]))
            # Choose the first (lowest) valid point
            for name, tensor in points:
                kp_x, kp_y, kp_conf = tensor
                kp_x, kp_y = self.homography_transform((kp_x, kp_y))
                if kp_conf > self.conf_thresh and 0.0 <= kp_y < self.y_axis_max:
                    kp_list.append([kp_x, kp_y])
                    break
        return kp_list if kp_list else None

    def homography_transform(self, pixel):
        image_point = np.array([pixel[0], pixel[1], 1], dtype=np.float32)
        ground_point = np.dot(self.H, image_point)
        ground_point /= ground_point[2]
        return ground_point[:2]
    
    def update_battery(self):
        try:
            self.battery_voltage = self.uart.get_battery_voltage()
            self.get_logger().info(f"Battery voltage: {self.battery_voltage} V")

            if self.battery_voltage > 37.0:
                self.led_ring.flash_color(0, 255, 0, 1.0)
            elif self.battery_voltage < 30.0:
                self.led_ring.flash_color(255, 0, 0, 1.0)
            else:
                self.led_ring.flash_color(255, 255, 0, 1.0)

        except Exception as e:
            self.get_logger().error(f"Error getting battery voltage: {e}")

    def publish_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_publisher.publish(msg)

    # -------------------------
    # CALLBACKS
    # -------------------------
    def cmd_callback(self, msg):
        command_map = {
            "start": State.EXPLORING,
            "stop": State.IDLE,
            "goal_reached": State.ALIGNING,
        }

        if msg.data in command_map:
            self.transition_to_state(command_map[msg.data])
        elif msg.data == "get_img":
            self.cv.capture_and_save_image()
        elif msg.data == "print_odom":
            if self.odom:
                self.get_logger().info(f"Odometry x: {self.odom.pose.pose.position.x}")
            else:
                self.get_logger().info("No odometry received yet.")
        elif msg.data == "battery":
            self.update_battery()
        else:
            self.get_logger().error(f"'{msg.data}' is not a valid command.")

    def odom_callback(self, msg):
        self.odom = msg

    # -------------------------
    # STATE TRANSITION HANDLING
    # -------------------------
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
        # Cancel any timers that might be running
        if self.explore_timer is not None:
            self.explore_timer.cancel()
            self.explore_timer = None
        if self.align_timer is not None:
            self.align_timer.cancel()
            self.align_timer = None
        if self.wait_timer is not None:
            self.wait_timer.cancel()
            self.wait_timer = None

        # Start the appropriate state handler
        if self.state == State.EXPLORING:
            self.start_exploring()
        elif self.state == State.IDLE:
            self.publish_twist(0, 0)
        elif self.state == State.ALIGNING:
            self.start_aligning()
        elif self.state == State.WAITING:
            self.start_waiting()

    def destroy_node(self):
        if self.explore_timer is not None:
            self.explore_timer.cancel()
        if self.align_timer is not None:
            self.align_timer.cancel()
        if self.wait_timer is not None:
            self.wait_timer.cancel()
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

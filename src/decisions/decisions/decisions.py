import cv2
import numpy as np
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from utils.uart import UART
from utils.nucleo_gpio import NucleoGPIO
from utils.neopixel_ring import NeoPixelRing
from decisions.yolo_model import YOLOModel
from utils.robot_params import (
    y_axis_max,
    y_axis_alignment_tolerance,
    pixel_points,
    ground_points,
    explore_linear_speed,
)


class State(Enum):
    IDLE = auto()
    EXPLORING = auto()
    ALIGNING = auto()
    WAITING = auto()


class DecisionsNode(Node):
    def __init__(self):
        super().__init__("decisions_node")

        # Subscribers & Publishers
        self.create_subscription(String, "/cmd", self.cmd_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 1)
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cmd_odom_publisher = self.create_publisher(Odometry, "/cmd_odom", 10)
        self.last_odom = None

        # Hardware and utility components
        self.uart = UART()
        self.led_ring = NeoPixelRing()
        self.nuc_gpio = NucleoGPIO()
        self.cv_model = YOLOModel()
        self.kp_conf_thresh = 0.6
        self.box_conf_thresh = 0.5

        # Robot parameters
        self.y_axis_alignment_tolerance = y_axis_alignment_tolerance
        self.y_axis_max = y_axis_max
        self.move_timeout = 5  # seconds
        self.battery_voltage = None
        self.update_battery()

        # Timers
        self.explore_timer = None
        self.align_timer = None
        self.wait_timer = None

        # Compute homography matrix
        self.H, status = cv2.findHomography(pixel_points, ground_points, cv2.RANSAC, 5.0)
        if self.H is not None:
            self.get_logger().info(f"Homography inliers: {status.ravel()}")
        else:
            self.get_logger().error("Homography computation failed.")

        # Allowed state transitions
        self.valid_transitions = {
            State.IDLE: [State.EXPLORING],
            State.EXPLORING: [State.IDLE, State.ALIGNING],
            State.ALIGNING: [State.IDLE, State.WAITING],
            State.WAITING: [State.IDLE, State.EXPLORING, State.ALIGNING],
        }
        self.state = State.IDLE

        self.led_ring.set_color(255, 255, 255, 1.0)
        self.get_logger().info("Decisions Node Initialized")

    def cancel_all_timers(self):
        """Cancel any running timers."""
        for attr in ("explore_timer", "align_timer", "wait_timer"):
            timer = getattr(self, attr)
            if timer is not None:
                timer.cancel()
                setattr(self, attr, None)

    def on_state_entry(self):
        """Called after a state transition; cancels any active timers and starts state-specific logic."""
        self.cancel_all_timers()
        if self.state == State.EXPLORING:
            self.start_exploring()
        elif self.state == State.IDLE:
            self.publish_twist(0, 0)
        elif self.state == State.ALIGNING:
            self.start_aligning()
        elif self.state == State.WAITING:
            self.start_waiting()

    def transition_to_state(self, new_state: State):
        """Transition to a new state if allowed."""
        if new_state == self.state:
            self.get_logger().error(f"Already in state {self.state.name}")
            return

        if new_state not in self.valid_transitions.get(self.state, []):
            self.get_logger().error(
                f"Invalid transition from {self.state.name} to {new_state.name}"
            )
            return

        self.get_logger().info(
            f"Transitioning from {self.state.name} to {new_state.name}"
        )
        self.state = new_state
        self.on_state_entry()

    def start_exploring(self):
        """Begin exploring: move forward and set a timer to check for boxes."""
        self.publish_twist(explore_linear_speed, 0)
        if self.explore_timer:
            self.explore_timer.cancel()
        self.explore_timer = self.create_timer(0.1, self.explore_callback)

    def explore_callback(self):
        if self.state != State.EXPLORING:
            if self.explore_timer:
                self.explore_timer.cancel()
            return

        boxes = self.get_boxes(save=True)
        if boxes is not None:
            self.explore_timer.cancel()
            self.transition_to_state(State.ALIGNING)

    def start_aligning(self):
        """Stop movement and set a timer to adjust alignment."""
        self.publish_twist(0, 0)
        if self.align_timer:
            self.align_timer.cancel()
        self.align_timer = self.create_timer(0.1, self.align_callback)

    def align_callback(self):
        if self.state != State.ALIGNING:
            if self.align_timer:
                self.align_timer.cancel()
            return

        kp_list = self.get_keypoints()
        if kp_list is None:
            return

        # Select keypoint with x-coordinate closest to zero
        best_point = min(kp_list, key=lambda pt: abs(pt[0]))
        # Check if aligned (convert mm to m)
        if abs(best_point[0] / 1000.0) < self.y_axis_alignment_tolerance:
            self.get_logger().info("Y-axis aligned. Removing flower.")
            self.uart.send_command(1, best_point[1])
            self.align_timer.cancel()
            self.transition_to_state(State.WAITING)
            return

        if self.last_odom is None:
            self.get_logger().warn("Odometry not yet received.")
            return

        # Adjust odometry based on keypoint displacement
        new_odom = Odometry()
        new_odom.header = self.last_odom.header
        new_odom.child_frame_id = self.last_odom.child_frame_id
        new_odom.pose.pose.position.x = (
            self.last_odom.pose.pose.position.x + (best_point[0] / 1000.0)
        )
        new_odom.pose.pose.position.y = self.last_odom.pose.pose.position.y
        self.get_logger().info(
            f"Adjusting position to x: {new_odom.pose.pose.position.x}"
        )
        self.cmd_odom_publisher.publish(new_odom)
        self.align_timer.cancel()
        self.transition_to_state(State.WAITING)

    def start_waiting(self):
        """Start a timer to wait for the move to complete."""
        self.wait_start_time = self.get_clock().now()
        if self.wait_timer:
            self.wait_timer.cancel()
        self.wait_timer = self.create_timer(0.1, self.wait_callback)

    def wait_callback(self):
        if self.state != State.WAITING:
            if self.wait_timer:
                self.wait_timer.cancel()
            return

        elapsed = (self.get_clock().now() - self.wait_start_time).nanoseconds / 1e9
        if elapsed > self.move_timeout:
            self.get_logger().error("Timeout waiting for move.")
            self.wait_timer.cancel()
            self.transition_to_state(State.IDLE)

    def get_boxes(self, save=False):
        """Run inference and filter detected boxes based on confidence."""
        result = self.cv_model.run_inference(save=save)
        if result is None:
            return None

        valid = result.boxes.conf >= self.box_conf_thresh
        result.boxes = result.boxes[valid]
        return result.boxes if len(result.boxes) > 0 else None

    def get_keypoints(self, save=False):
        """Run inference and return valid keypoints after homography transformation."""
        result = self.cv_model.run_inference(save=save)
        if result is None or not result.keypoints:
            return None

        kp_list = []
        for kp in result.keypoints:
            # Pair names with keypoint data and sort by preferred order
            names = ["flower", "base", "upper", "lower"]
            points = list(zip(names, kp.data[0]))
            points.sort(key=lambda x: ["base", "lower", "upper", "flower"].index(x[0]))
            for name, tensor in points:
                kp_x, kp_y, kp_conf = tensor
                kp_x, kp_y = self.homography_transform((kp_x, kp_y))
                if kp_conf > self.kp_conf_thresh and 0.0 <= kp_y < self.y_axis_max:
                    kp_list.append([kp_x, kp_y])
                    break
        return kp_list if kp_list else None

    def homography_transform(self, pixel):
        """Transform a point from pixel to ground coordinates using homography."""
        pt = np.array([pixel[0], pixel[1], 1], dtype=np.float32)
        ground = np.dot(self.H, pt)
        ground /= ground[2]
        return ground[:2]

    def update_battery(self):
        """Update battery voltage and flash LED ring accordingly."""
        try:
            self.battery_voltage = self.uart.get_battery_voltage()
            self.get_logger().info(f"Battery voltage: {self.battery_voltage} V")

            if self.battery_voltage > 37.0:  # Good
                self.led_ring.flash_color(0, 255, 0, 1.0)
            elif self.battery_voltage < 30.0:  # Bad
                self.led_ring.flash_color(255, 0, 0, 1.0)
            else:  # OK
                self.led_ring.flash_color(255, 255, 0, 1.0)
        except Exception as e:
            self.get_logger().error(f"Error getting battery voltage: {e}")

    def publish_twist(self, linear, angular):
        """Publish a Twist message to control robot movement."""
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_publisher.publish(msg)

    def cmd_callback(self, msg: String):
        """Handle incoming commands."""
        command = msg.data.lower()
        if command == "start":
            self.transition_to_state(State.EXPLORING)
        elif command == "stop":
            self.transition_to_state(State.IDLE)
        elif command == "goal_reached":
            self.transition_to_state(State.ALIGNING)
        elif command == "get_img":
            self.cv_model.capture_and_save_image()
        elif command == "print_odom":
            if self.last_odom:
                pos = self.last_odom.pose.pose.position
                orient = self.last_odom.pose.pose.orientation
                self.get_logger().info(
                    f"(x, y, z_or): ({pos.x}, {pos.y}, {orient.z})"
                )
            else:
                self.get_logger().info("No odometry received yet.")
        elif command == "battery":
            self.update_battery()
        else:
            self.get_logger().error(f"'{command}' is not a valid command.")

    def odom_callback(self, msg: Odometry):
        """Store the last received odometry message."""
        self.last_odom = msg

    def destroy_node(self):
        self.cancel_all_timers()
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


if __name__ == "__main__":
    main()

import cv2
import numpy as np
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from std_msgs.msg import Float32, Float32MultiArray, MultiArrayLayout, MultiArrayDimension, String, UInt8MultiArray


from utils.nucleo_gpio import NucleoGPIO
from utils.neopixel_ring import NeoPixelRing
from decisions.yolo_model import YOLOModel
from decisions.planner import Planner
import utils.robot_params as rp
from utils.utilities import two_d_array_to_float32_multiarray, package_removal_command

class State(Enum):
    IDLE = auto()
    EXPLORING = auto()
    ALIGNING = auto()
    WAITING = auto()

class DecisionsNode(Node):
    def __init__(self):
        super().__init__("decisions_node")

        self.create_subscription(String, "/cmd", self.cmd_callback, 10)
        self.create_subscription(PoseStamped, "/pose", self.pose_callback, 1)
        self.create_subscription(Float32, "/battery", self.update_battery, 10)
        
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.path_pub = self.create_publisher(Float32MultiArray, "/path", 10)
        self.positioning_pub = self.create_publisher(Float32MultiArray, "/position", 10)
        self.uart_pub = self.create_publisher(UInt8MultiArray, "/send_uart", 10)
        self.pose = None

        # Hardware and utility components
        self.led_ring = NeoPixelRing()
        self.nuc_gpio = NucleoGPIO()
        self.cv_model = YOLOModel()
        self.kp_conf_thresh = 0.6
        self.box_conf_thresh = 0.5
        self.planner = Planner()
        self.path = None

        self.move_timeout = 5
        self.battery_voltage = None
        self.request_battery_voltage()

        # Compute homography matrix
        self.H, status = cv2.findHomography(rp.pixel_points, rp.ground_points, cv2.RANSAC, 5.0)
        if self.H is not None:
            self.get_logger().info(f"Homography inliers: {status.ravel()}")
        else:
            self.get_logger().error("Homography computation failed.")

        # State timers
        self.explore_timer = None
        self.align_timer = None
        self.wait_timer = None
        self.removal_in_process = False

        self.valid_transitions = {
            State.IDLE: [State.EXPLORING],
            State.EXPLORING: [State.IDLE, State.ALIGNING],
            State.ALIGNING: [State.IDLE, State.WAITING],
            State.WAITING: [State.IDLE, State.EXPLORING, State.ALIGNING],
        }
        self.state = State.IDLE
        
        self.get_logger().info("Decisions Initialized")

    def cancel_all_timers(self):
        for attr in ("explore_timer", "align_timer", "wait_timer"):
            timer = getattr(self, attr)
            if timer is not None:
                timer.cancel()
                setattr(self, attr, None)

    def on_state_entry(self):
        self.cancel_all_timers()
        if self.state == State.EXPLORING:
            self.start_exploring()
        elif self.state == State.IDLE:
            self.led_ring.off()
            self.path_pub.publish(Float32MultiArray())
            self.positioning_pub.publish(Float32MultiArray())
            self.publish_twist(0, 0)
        elif self.state == State.ALIGNING:
            self.start_aligning()
        elif self.state == State.WAITING:
            self.start_waiting()

    def transition_to_state(self, new_state: State):
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
        self.led_ring.set_color(255, 255, 255, 1.0)

        self.path = two_d_array_to_float32_multiarray(self.planner.plan())
        self.path_pub.publish(self.path)

        if self.explore_timer:
            self.explore_timer.cancel()
        self.explore_timer = self.create_timer(0.1, self.explore_callback)

    def explore_callback(self):
        if self.state != State.EXPLORING:
            if self.explore_timer:
                self.explore_timer.cancel()
            return

        boxes = self.get_boxes()
        if boxes is not None:
            self.explore_timer.cancel()
            self.transition_to_state(State.ALIGNING)

    def start_aligning(self):
        self.led_ring.set_color(255, 255, 255, 1.0)
        
        self.path_pub.publish(Float32MultiArray())

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
        if abs(best_point[0] / 1000.0) < rp.y_axis_alignment_tolerance:
            self.get_logger().info("Y-axis aligned. Removing flower.")
            self.uart_pub.publish(package_removal_command(best_point[1]))
            self.align_timer.cancel()
            self.transition_to_state(State.WAITING)
            return

        x = self.pose.pose.position.x + best_point[0] / 1000.0
        y = self.pose.pose.position.y
        z = self.pose.pose.orientation.z
        new_position = two_d_array_to_float32_multiarray([[x, y, z]])

        self.positioning_pub.publish(new_position)
        
        self.align_timer.cancel()
        self.transition_to_state(State.WAITING)

    def start_waiting(self):
        self.wait_start_time = self.get_clock().now()
        if self.wait_timer:
            self.wait_timer.cancel()
        self.wait_timer = self.create_timer(0.1, self.wait_callback)

    def wait_callback(self):
        if self.state != State.WAITING:
            if self.wait_timer:
                self.wait_timer.cancel()
            return

        if self.removal_in_process:
            self.led_ring.step_animation()


    def get_boxes(self):
        result = self.cv_model.run_inference()
        if result is None:
            return None

        valid = result.boxes.conf >= self.box_conf_thresh
        result.boxes = result.boxes[valid]
        return result.boxes if len(result.boxes) > 0 else None

    def get_keypoints(self):
        result = self.cv_model.run_inference()
        if result is None or not result.keypoints:
            return None

        kp_list = []
        for kp in result.keypoints:
            names = ["flower", "base", "upper", "lower"]
            points = list(zip(names, kp.data[0]))
            points.sort(key=lambda x: ["base", "lower", "upper", "flower"].index(x[0]))
            for name, tensor in points:
                kp_x, kp_y, kp_conf = tensor
                kp_x, kp_y = self.homography_transform((kp_x, kp_y))
                if kp_conf > self.kp_conf_thresh and 0.0 <= kp_y < rp.y_axis_max:
                    kp_list.append([kp_x, kp_y])
                    break
        return kp_list if kp_list else None

    def homography_transform(self, pixel):
        pt = np.array([pixel[0], pixel[1], 1], dtype=np.float32)
        ground = np.dot(self.H, pt)
        ground /= ground[2]
        return ground[:2]

    def request_battery_voltage(self):
        msg = UInt8MultiArray()
        msg.data = [rp.battery_byte]
        self.uart_pub.publish(msg)

    def update_battery(self, msg):
        self.battery_voltage = msg.data
        try:
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

    def cmd_callback(self, msg: String):
        command = msg.data.lower()
        if command == "start":
            self.transition_to_state(State.EXPLORING)
        elif command == "stop":
            self.transition_to_state(State.IDLE)
        elif command == "new_pos_reached":
            self.transition_to_state(State.ALIGNING)
        elif command == "path_complete":
            self.transition_to_state(State.IDLE)
        elif command == "removal_complete":
            self.transition_to_state(State.EXPLORING)
        elif command == "get_img":
            self.cv_model.capture_and_save_image()
        elif command == "print_pose":
            if self.pose:
                pos = self.pose.pose.position
                orient = self.pose.pose.orientation
                self.get_logger().info(
                    f"(x, y, z_or): ({pos.x}, {pos.y}, {orient.z})"
                )
            else:
                self.get_logger().info("No pose received yet.")
        elif command == "battery":
            self.request_battery_voltage()
        else:
            self.get_logger().error(f"'{command}' is not a valid command.")

    def pose_callback(self, msg):
        self.pose = msg

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

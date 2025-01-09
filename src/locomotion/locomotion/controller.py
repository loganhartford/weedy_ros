#!/mnt/shared/weedy_ros/src/locomotion/locomotion/pwm_venv/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from motor_control import MotorController

class ControllerNode(Node):
    def __init__(self):
        super().__init__('Controller')
        self.cmd_vel_subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.cmd_pose_subscription = self.create_subscription(PoseStamped, '/cmd_pose', self.cmd_pose_callback, 10)
        self.motor_controller = MotorController()

    def cmd_vel_callback(self, msg):
        self.motor_controller.set_velocity(msg.linear.x, msg.angular.z)

    def cmd_pose_callback(self, msg):
        pass

    def destroy_node(self):
        self.motor_controller.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import time
from rpi_hardware_pwm import HardwarePWM

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control')

        self.cmd_vel_subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.motor1 = HardwarePWM(pwm_channel=0, hz=20000, chip=2)
        self.motor2 = HardwarePWM(pwm_channel=1, hz=20000, chip=2)
        self.motor1.start(0)
        self.motor2.start(0)
    
    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        duty_cycle1 = linear_x + angular_z
        duty_cycle2 = linear_x - angular_z

        self.motor1.change_duty_cycle(duty_cycle1)
        self.motor2.change_duty_cycle(duty_cycle2)
    
    def destroy_node(self):
        self.motor1.stop()
        self.motor2.stop()
        super().destroy_node()
    
def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
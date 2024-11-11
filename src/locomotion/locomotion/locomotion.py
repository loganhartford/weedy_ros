import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import logging

class DifferentialDriveNode(Node):
    def __init__(self):
        super().__init__('differential_drive_node')
        
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.logger = logging.getLogger('DifferentialDriveNode')
        logging.basicConfig(level=logging.INFO)

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        self.logger.info(f'Received cmd_vel - Linear X: {linear_x}, Angular Z: {angular_z}')

def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDriveNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('Localization')
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.current_pose = PoseStamped()
        self.current_twist = Twist()

        # Periodically publish odometry
        self.timer = self.create_timer(0.1, self.publish_odom)

    def update_odometry(self, pose: PoseStamped, twist: Twist):
        self.current_pose = pose
        self.current_twist = twist

    def get_current_odometry(self):
        return self.current_pose, self.current_twist

    def publish_odom(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.pose.pose = self.current_pose
        odom_msg.twist.twist = self.current_twist
        self.odom_publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

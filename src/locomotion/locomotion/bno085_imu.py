import time
import board
import busio
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ROTATION_VECTOR
)
from adafruit_bno08x.i2c import BNO08X_I2C

class BNO085IMU(Node):
    def __init__(self, frequency=50):
        super().__init__('bno085_imu_node')
        
        # Initialize I2C
        self.i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        self.bno = BNO08X_I2C(self.i2c)

        # Enable IMU features
        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

        # Create IMU publisher
        self.imu_publisher = self.create_publisher(Imu, '/imu', 10)

        # Set up timer to publish data
        self.timer_period = 1.0 / frequency  # Convert frequency to period
        self.timer = self.create_timer(self.timer_period, self.publish_imu_data)

    def publish_imu_data(self):
        """Read IMU data and publish as ROS 2 IMU message"""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        # Read accelerometer (m/s^2)
        accel_x, accel_y, accel_z = self.bno.acceleration
        msg.linear_acceleration.x = accel_x
        msg.linear_acceleration.y = accel_y
        msg.linear_acceleration.z = accel_z

        # Read gyroscope (rad/s)
        gyro_x, gyro_y, gyro_z = self.bno.gyro
        msg.angular_velocity.x = gyro_x
        msg.angular_velocity.y = gyro_y
        msg.angular_velocity.z = gyro_z

        # Read quaternion rotation
        quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
        msg.orientation.x = quat_i
        msg.orientation.y = quat_j
        msg.orientation.z = quat_k
        msg.orientation.w = quat_real

        # Publish the IMU message
        self.imu_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    imu_node = BNO085IMU(frequency=50)
    try:
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        pass
    finally:
        imu_node.destroy_node()

if __name__ == '__main__':
    main()

import board
import busio
import rclpy
import lgpio
import time
import math

from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_MAGNETOMETER
)
from adafruit_bno08x.i2c import BNO08X_I2C

from rclpy.node import Node
from sensor_msgs.msg import Imu

from utils.utilities import quaternion_multiply


class BNO085IMU(Node):
    def __init__(self):
        super().__init__('bno085_imu_node')
        
        self.imu_publisher = self.create_publisher(Imu, '/imu', 10)
        
        self.frequency = 50
        self.frame_id = "base_link"

        self.reset_pin = 23
        self.chip = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.chip, self.reset_pin, level=1)

        self.q_offset = [0, 0, -0.7071, 0.7071] # Rotated -90 about z
        
        self.bno = None
        self.init_bno085()

        self.timer_period = 1.0 / self.frequency  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_imu_data)

        self.get_logger().info("BNO085 IMU Initialized")
    
    def init_bno085(self):
        while self.bno is None:
            try:
                self.i2c = busio.I2C(board.SCL, board.SDA)
                self.bno = BNO08X_I2C(self.i2c)

                self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
                self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
                self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            except Exception as e:
                self.bno = None
                self.get_logger().error(f"Error initializing BNO085 IMU: {e}")
                self.reset()
                time.sleep(1)

    def publish_imu_data(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        try:
            # Read accelerometer data (m/s^2)
            accel_x, accel_y, accel_z = self.bno.acceleration
            msg.linear_acceleration.x = accel_y
            msg.linear_acceleration.y = -accel_x
            msg.linear_acceleration.z = accel_z

            # Read gyroscope data (rad/s)
            gyro_x, gyro_y, gyro_z = self.bno.gyro
            msg.angular_velocity.x = gyro_y
            msg.angular_velocity.y = -gyro_x
            msg.angular_velocity.z = gyro_z

            # Read quaternion rotation data
            quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
            q_sensor = [quat_i, quat_j, quat_k, quat_real]
            
            # Transform sensor quaternion into robot frame
            q_robot = quaternion_multiply(self.q_offset, q_sensor)

            msg.orientation.x = q_robot[0]
            msg.orientation.y = q_robot[1]
            msg.orientation.z = q_robot[2]
            msg.orientation.w = q_robot[3]
            
        except Exception as e:
            self.get_logger().error(f"Error reading IMU data: {e}")
            self.reset()
            return
        
        # Set covariance arrays (using -1 in the first element to indicate unknown covariance)
        msg.orientation_covariance = [-1.0] + [0.0] * 8
        msg.angular_velocity_covariance = [-1.0] + [0.0] * 8
        msg.linear_acceleration_covariance = [-1.0] + [0.0] * 8

        self.imu_publisher.publish(msg)
    
    def reset(self):
        lgpio.gpio_write(self.chip, self.reset_pin, 0)
        time.sleep(0.1)
        lgpio.gpio_write(self.chip, self.reset_pin, 1)
        time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    imu_node = BNO085IMU()
    try:
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        pass
    finally:
        imu_node.destroy_node()

if __name__ == '__main__':
    main()

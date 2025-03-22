import board
import busio
import rclpy
import lgpio
import time

from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_RAW_ACCELEROMETER,
    BNO_REPORT_RAW_GYROSCOPE,
    BNO_REPORT_RAW_MAGNETOMETER,
    BNO_REPORT_GAME_ROTATION_VECTOR,
    BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Bool
from geometry_msgs.msg import Quaternion

import utils.robot_params as rp


class BNO085IMU(Node):
    def __init__(self):
        super().__init__('bno085_imu_node')
        
        self.imu_publisher = self.create_publisher(Imu, '/imu/data', 10)
        if rp.POST_RAW:
            self.imu_raw_publisher = self.create_publisher(Imu, '/imu/raw', 10)
        if rp.USE_MAG:
            self.mag_publisher = self.create_publisher(MagneticField, '/mag/data', 10)
            self.mag_quat_publisher = self.create_publisher(Imu, '/mag/quat', 10)
            if rp.POST_RAW:
                self.mag_raw_publisher = self.create_publisher(MagneticField, '/mag/raw', 10)
        self.odom_reset_sub = self.create_subscription(Bool, '/reset_odom', self.reset_odom_callback, 1)
        
        self.frequency = 50
        self.frame_id = "imu_link"

        self.reset_pin = 23
        self.chip = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.chip, self.reset_pin, level=1)
        
        self.bno = None
        self.init_bno085()

        self.timer_period = 1.0 / self.frequency  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_imu_data)

        self.get_logger().info("BNO085 IMU Initialized")
    
    def re_init_imu(self):
        self.reset()
        self.bno = None
        self.init_bno085()

    def init_bno085(self):
        while self.bno is None:
            try:
                self.i2c = busio.I2C(board.SCL, board.SDA)
                self.bno = BNO08X_I2C(self.i2c)

                self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
                self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
                self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

                if rp.POST_RAW:
                    self.bno.enable_feature(BNO_REPORT_RAW_ACCELEROMETER)
                    self.bno.enable_feature(BNO_REPORT_RAW_GYROSCOPE)
                    self.bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)

                if rp.USE_MAG:
                    self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
                    self.bno.enable_feature(BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)
                    if rp.POST_RAW:
                        self.bno.enable_feature(BNO_REPORT_RAW_MAGNETOMETER)

            except Exception as e:
                self.bno = None
                self.get_logger().error(f"Error initializing BNO085 IMU: {e}")
                self.reset()
        
        num_calibrations = 50
        

    def publish_imu_data(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id

        if rp.POST_RAW:
            imu_raw_msg = Imu()
            imu_raw_msg.header.stamp = imu_msg.header.stamp
            imu_raw_msg.header.frame_id = imu_msg.header.frame_id
        
        if rp.USE_MAG:
            mag_msg = MagneticField()
            mag_msg.header.stamp = imu_msg.header.stamp
            mag_msg.header.frame_id = imu_msg.header.frame_id

            mag_quat_msg = Imu()
            mag_quat_msg.header.stamp = imu_msg.header.stamp
            mag_quat_msg.header.frame_id = imu_msg.header.frame_id

            if rp.POST_RAW:
                mag_raw_msg = MagneticField()
                mag_raw_msg.header.stamp = imu_msg.header.stamp
                mag_raw_msg.header.frame_id = imu_msg.header.frame_id

        try:
            # Read calibrated IMU data
            # Accelerometer data (m/s^2)
            accel_x, accel_y, accel_z = self.bno.acceleration
            imu_msg.linear_acceleration.x = float(accel_x)
            imu_msg.linear_acceleration.y = float(accel_y)
            imu_msg.linear_acceleration.z = float(accel_z)

            # Gyroscope data (rad/s)
            gyro_x, gyro_y, gyro_z = self.bno.gyro
            imu_msg.angular_velocity.x = float(gyro_x)
            imu_msg.angular_velocity.y = float(gyro_y)
            imu_msg.angular_velocity.z = float(gyro_z)

            # Quaternion rotation data
            quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
            imu_msg.orientation.x = float(quat_i)
            imu_msg.orientation.y = float(quat_j)
            imu_msg.orientation.z = float(quat_k)
            imu_msg.orientation.w = float(quat_real)

            self.imu_publisher.publish(imu_msg)

            # Read raw IMU data
            if rp.POST_RAW:
                # Raw accelerometer data (m/s^2)
                accel_x, accel_y, accel_z = self.bno.raw_acceleration
                imu_raw_msg.linear_acceleration.x = float(accel_x)
                imu_raw_msg.linear_acceleration.y = float(accel_y)
                imu_raw_msg.linear_acceleration.z = float(accel_z)

                # Raw gyroscope data (rad/s)
                gyro_x, gyro_y, gyro_z = self.bno.raw_gyro
                imu_raw_msg.angular_velocity.x = float(gyro_x)
                imu_raw_msg.angular_velocity.y = float(gyro_y)
                imu_raw_msg.angular_velocity.z = float(gyro_z)

                # Raw quaternion rotation data
                quat_i, quat_j, quat_k, quat_real = self.bno.game_quaternion
                imu_raw_msg.orientation.x = float(quat_i)
                imu_raw_msg.orientation.y = float(quat_j)
                imu_raw_msg.orientation.z = float(quat_k)
                imu_raw_msg.orientation.w = float(quat_real)

                self.imu_raw_publisher.publish(imu_raw_msg)

            # Read magnetometer data (uT)
            if rp.USE_MAG:
                # Calibrated magnetometer data
                mag = self.bno.magnetic
                if mag is not None:
                    mag_x, mag_y, mag_z = mag
                    mag_msg.magnetic_field.x = float(mag_x)
                    mag_msg.magnetic_field.y = float(mag_y)
                    mag_msg.magnetic_field.z = float(mag_z)
                    self.mag_publisher.publish(mag_msg)

                # Magnetometer quaternion data
                mag_quat = self.bno.geomagnetic_quaternion
                if mag_quat is not None:
                    mag_quat_i, mag_quat_j, mag_quat_k, mag_quat_real = mag_quat
                    mag_quat_msg.orientation.x = float(mag_quat_i)
                    mag_quat_msg.orientation.y = float(mag_quat_j)
                    mag_quat_msg.orientation.z = float(mag_quat_k)
                    mag_quat_msg.orientation.w = float(mag_quat_real)
                    self.mag_quat_publisher.publish(mag_quat_msg)

                if rp.POST_RAW:
                    # Raw magnetometer data
                    raw_mag = self.bno.raw_magnetic
                    if raw_mag is not None:
                        mag_x, mag_y, mag_z = raw_mag
                        mag_raw_msg.magnetic_field.x = float(mag_x)
                        mag_raw_msg.magnetic_field.y = float(mag_y)
                        mag_raw_msg.magnetic_field.z = float(mag_z)
                        self.mag_raw_publisher.publish(mag_raw_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error reading IMU data: {e}")
            self.re_init_imu()
            return
    
    def reset_odom_callback(self, msg):
        if msg.data:
            self.re_init_imu()

    def reset(self):
        lgpio.gpio_write(self.chip, self.reset_pin, 0)
        time.sleep(0.5)
        lgpio.gpio_write(self.chip, self.reset_pin, 1)
        time.sleep(0.5)

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

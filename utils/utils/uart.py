import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, UInt8MultiArray, String, Int32MultiArray
import serial
import time
import numpy as np

from utils.exceptions import UARTError
from utils.nucleo_gpio import NucleoGPIO
import utils.robot_params as rp

class UARTNode(Node):
    def __init__(self):
        super().__init__('uart_node')

        self.ser = serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=0.1)
        self.ack_timeout = 0.1     # seconds
        self.num_tick_timeouts = 0

        self.subscription = self.create_subscription(
            UInt8MultiArray,
            '/send_uart',
            self.bytes_callback,
            10
        )

        self.cmd_pub = self.create_publisher(String, '/cmd', 10)
        self.ticks_pub = self.create_publisher(Int32MultiArray, '/ticks', 1)
        self.battery_pub = self.create_publisher(Float32, '/battery', 1)
        self.create_timer(0.01, self.check_incoming_messages)

        self.nucleo_gpio = NucleoGPIO() # For resetting the Nucleo

        self.get_logger().info("UART Initialized")

    def bytes_callback(self, msg):
        data = bytes(msg.data)
        if not data:
            self.get_logger().warning("Received empty byte array.")
            return

        if data[0] == rp.weed_removal_byte:
            self.send_weed_removal(data)
        elif data[0] == rp.battery_byte:
            self.get_battery_voltage()
        else:
            self.ser.write(data)

    def send_weed_removal(self, byte_list):
        if len(byte_list) != 3:
            raise UARTError("Invalid command message length.")
        checksum = sum(byte_list) % 256
        message = byte_list + bytes([checksum])

        for attempt in range(3):
            self.ser.write(message)
            if self.wait_for_acknowledgment():
                return
            else:
                self.nucleo_gpio.toggle_reset()

        self.get_logger().error(f"UART Error during command processing: Timeout waiting for acknowledgment after 3 attempts.")
    
    def get_battery_voltage(self):
        self.ser.write(bytes([rp.battery_byte]))

        buffer = bytearray()
        start_time = time.time()
        while len(buffer) < 4:
            if (time.time() - start_time) > self.ack_timeout:
                raise UARTError("Timeout waiting for battery reply.")
            data = self.ser.read(self.ser.in_waiting or 1)
            if data:
                buffer.extend(data)

        if buffer[0] != rp.battery_byte:
            raise UARTError("Invalid start byte in battery reply.")

        if sum(buffer[:-1]) % 256 != buffer[-1]:
            raise UARTError("Invalid battery checksum.")

        int_part = buffer[1]
        decimal_part = buffer[2]
        voltage = int_part + (decimal_part / 100)

        self.battery_pub.publish(Float32(data=voltage))

    def wait_for_acknowledgment(self):
        start_time = time.time()
        while (time.time() - start_time) < self.ack_timeout:
            if self.ser.in_waiting:
                data = self.ser.read(self.ser.in_waiting)
                if data and data[0] == rp.ack_byte: 
                    return True
            time.sleep(0.05)
        return False

    def check_incoming_messages(self):
        if self.ser.in_waiting:
            buffer = self.ser.read(self.ser.in_waiting)
            
            # Process ticks message
            if len(buffer) >= 6 and buffer[0] == rp.ticks_byte:
                
                if (sum(buffer[0:5]) % 256) != buffer[5]:
                    self.get_logger().error("Invalid tick checksum.")
                    return

                # Parse tick values (handle signed 16-bit integers)
                ticks1 = (buffer[1] << 8) | buffer[2]
                ticks2 = (buffer[3] << 8) | buffer[4]
                if ticks1 & (1 << 15):
                    ticks1 -= (1 << 16)
                if ticks2 & (1 << 15):
                    ticks2 -= (1 << 16)
                
                ticks_msg = Int32MultiArray()
                ticks_msg.data = [ticks1, ticks2]
                self.ticks_pub.publish(ticks_msg)

            elif buffer[0] == rp.callback_byte:
                cmd_msg = String(data="removal_complete")
                self.cmd_pub.publish(cmd_msg)
            else:
                self.get_logger().warn("Received unknown message type: " + str(buffer))

def main(args=None):
    rclpy.init(args=args)
    uart_node = UARTNode()
    try:
        rclpy.spin(uart_node)
    except KeyboardInterrupt:
        pass
    finally:
        uart_node.destroy_node()

if __name__ == '__main__':
    main()

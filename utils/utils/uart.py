import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from std_msgs.msg import String, UInt8MultiArray, String, Int32MultiArray
import serial
import time
import numpy as np

from utils.exceptions import UARTError
from utils.nucleo_gpio import NucleoGPIO

class UARTNode(Node):
    def __init__(self):
        super().__init__('uart_node')

        self.ser = serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=0.1)
        self.clock = Clock()
        self.ticks_timeout = 0.01  # seconds
        self.ack_timeout = 0.1     # seconds
        self.num_tick_timeouts = 0

        self.subscription = self.create_subscription(
            UInt8MultiArray,
            '/send_uart',
            self.bytes_callback,
            10
        )

        self.cmd_pub = self.create_publisher(String, '/cmd', 10)
        self.ticks_pub = self.create_publisher(Int32MultiArray, '/ticks', 10)
        self.create_timer(0.1, self.check_incoming_messages)

        # Command and special byte definitions
        self.weed_removal_byte = 0x01
        self.callback_byte = 0x02
        self.ticks_byte = 0xAE
        self.battery_byte = 0x11

        self.nucleo_gpio = NucleoGPIO() # For resetting the Nucleo

    def bytes_callback(self, msg: UInt8MultiArray):
        data = bytes(msg.data)
        if not data:
            self.get_logger().warning("Received empty byte array.")
            return

        if data[0] == self.weed_removal_byte:
            try:
                self.send_weed_removal(data)
            except UARTError as e:
                self.get_logger().error(f"UART Error during command processing: {e}")
        else:
            self.ser.write(data)
            self.get_logger().info("Non-command bytes sent without waiting.")

    def send_weed_removal(self, byte_list):
        if len(byte_list) != 3:
            raise UARTError("Invalid command message length.")
        checksum = sum(byte_list) % 256
        message = bytes(byte_list, checksum)

        for attempt in range(3):
            self.ser.write(message)
            if self.wait_for_acknowledgment():
                break
            else:
                self.nucleo_gpio.toggle_reset()

        raise UARTError("Timeout waiting for acknowledgment after 3 attempts.")

    def wait_for_acknowledgment(self) -> bool:
        start_time = time.time()
        buffer = bytearray()
        while (time.time() - start_time) < self.ack_timeout:
            data = self.ser.read(self.ser.in_waiting or 1)
            if data:
                buffer.extend(data)
                # Process in chunks of 5 bytes
                while len(buffer) >= 5:
                    message = buffer[:5]
                    if message[0] == 0x03:  # ACK message type
                        return True
                    buffer = buffer[5:]
            else:
                time.sleep(0.05)
        return False

    def check_incoming_messages(self):
        if self.ser.in_waiting:
            buffer = self.ser.read(self.ser.in_waiting)
            
            # Process ticks message
            if len(buffer) >= 6 and buffer[0] == self.ticks_byte:
                
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
                stamp = self.clock.now().nanoseconds

                ticks_msg = Int32MultiArray()
                ticks_msg.data = [ticks1, ticks2, stamp]
                self.ticks_pub.publish(ticks_msg)

            elif buffer[0] == self.callback_byte:
                cmd_msg = String(data="removal_complete")
                self.cmd_pub.publish(cmd_msg)
                self.get_logger().info("Published cmd: removed")
            else:
                self.get_logger().warn("Received unknown message type: " + str(buffer))

def main(args=None):
    rclpy.init(args=args)
    uart_node = UARTNode()
    try:
        rclpy.spin(uart_node)
    except KeyboardInterrupt:
        uart_node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        uart_node.destroy_node()

if __name__ == '__main__':
    main()

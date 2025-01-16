import rclpy
from rclpy.node import Node
import serial
import time
import threading

from custom_msgs.msg import CartesianCmd
from std_msgs.msg import Int32MultiArray

class UartNode(Node):
    def __init__(self):
        super().__init__('uart_node')

        self.subscriber = self.create_subscription(CartesianCmd, "/cmd_cartesian", self.cmd_cartesian_callback, 10)
        self.state_publisher = self.create_publisher(CartesianCmd, "/cartesian_state", 10)
        self.tick_publisher = self.create_publisher(Int32MultiArray, "/ticks", 10)

        # Initialize the serial connection
        self.ser = serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=0.1)
        self.ack_timeout = 1 # s

    def get_ticks(self):
        if self.ser.in_waiting:
            self.ser.reset_input_buffer()
        
        byte_to_send = bytes([0xAE])  # Create a single-byte bytes object
        self.ser.write(byte_to_send)  # Write the byte to the serial port

        buffer = bytearray()
        while len(buffer) < 6:  # Expecting 6 bytes: start byte, 2 bytes per tick, and checksum
            data = self.ser.read(self.ser.in_waiting or 1)
            if data:
                buffer.extend(data)
        stamp = self.get_clock().now()
        
        # Validate start byte
        if buffer[0] != 0xAE:
            self.get_logger().warning("Start byte mismatch. Discarding message.")
            return "e", "e", stamp
        
        # Validate the checksum
        checksum = sum(buffer[:-1]) % 256
        if checksum != buffer[-1]:
            self.get_logger().warning("Checksum mismatch. Discarding message.")
            return -1, -1, stamp

        # Parse the ticks values
        ticks1 = (buffer[1] << 8) | buffer[2]
        ticks2 = (buffer[3] << 8) | buffer[4]

        # Handle signed 16-bit integers
        if ticks1 & (1 << 15):
            ticks1 -= (1 << 16)  # Convert to signed 16-bit integer
        if ticks2 & (1 << 15):
            ticks2 -= (1 << 16)

        print(f"ticks1: {ticks1}, ticks2 {ticks2}")

        return ticks1, ticks2, stamp


    def cmd_cartesian_callback(self, msg):
        message = construct_message(0x01, msg.axis, msg.position)
        self.ser.write(message)
        self.get_logger().info(f"Sent Command: Axis {msg.axis}, Position {msg.position}")

        # Wait for acknowledgment
        if not self.wait_for_acknowledgment():
            self.get_logger().warning("Timeout waiting for acknowledgment.")
            return

        # Wait for data message from Nucleo
        data_received, resp_axis, resp_position = self.wait_for_data_message()
        if data_received:
            # Publish the received data to /cartesian_state
            response_msg = CartesianCmd()
            response_msg.axis = resp_axis
            response_msg.position = resp_position
            self.state_publisher.publish(response_msg)
            self.get_logger().info(f"Received Data: Axis {resp_axis}, Position {resp_position}")
        else:
            response_msg = CartesianCmd()
            response_msg.axis = -1
            response_msg.position = -1
            self.state_publisher.publish(response_msg)
            self.get_logger().warning("Timeout waiting for data message from Nucleo.")

    def wait_for_acknowledgment(self):
        start_time = time.time()
        buffer = bytearray()

        while (time.time() - start_time) < self.ack_timeout:
            data = self.ser.read(self.ser.in_waiting or 1)
            if data:
                buffer.extend(data)
                # Try to parse complete messages
                while len(buffer) >= 5:
                    message = buffer[:5]
                    try:
                        message_type, resp_axis, resp_position = parse_message(message)
                        if message_type == 0x03:
                            buffer = buffer[5:]  # Remove processed message
                            return True
                        else:
                            self.get_logger().warning(f"Received unexpected message type: {message_type}")
                            buffer = buffer[5:]
                    except ValueError as e:
                        self.get_logger().error(f"Error parsing message: {e}")
                        buffer = buffer[1:]  # Remove one byte and retry
            else:
                time.sleep(0.1)

        return False

    def wait_for_data_message(self):
        start_time = time.time()
        buffer = bytearray()

        # while (time.time() - start_time) < self.data_timeout:
        while True: # For now, just wait indefinitely
            data = self.ser.read(self.ser.in_waiting or 1)
            if data:
                buffer.extend(data)
                # Try to parse complete messages
                while len(buffer) >= 5:
                    message = buffer[:5]
                    try:
                        message_type, resp_axis, resp_position = parse_message(message)
                        if message_type == 0x02:  # Data message type
                            buffer = buffer[5:]  # Remove processed message
                            return True, resp_axis, resp_position
                        else:
                            self.get_logger().warning(f"Received unexpected message type: {message_type}")
                            buffer = buffer[5:]
                    except ValueError as e:
                        self.get_logger().error(f"Error parsing message: {e}")
                        buffer = buffer[1:]  # Remove one byte and retry
            else:
                time.sleep(0.1)

        return False, None, None

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()

def construct_message(message_type, axis, position):
    pos_high = (position >> 8) & 0xFF
    pos_low = position & 0xFF
    checksum = (message_type + axis + pos_high + pos_low) % 256
    message = bytes([message_type, axis, pos_high, pos_low, checksum])
    
    return message

def parse_message(message):
    message_type = message[0]
    axis = message[1]
    pos_high = message[2]
    pos_low = message[3]
    checksum = message[4]
    
    calculated_checksum = (message_type + axis + pos_high + pos_low) % 256
    if checksum != calculated_checksum:
        raise ValueError("Checksum does not match.")
    
    position = (pos_high << 8) | pos_low
    
    return message_type, axis, position

def main(args=None):
    rclpy.init(args=args)
    node = UartNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()

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

        # Reading ticks
        self.polling_event = threading.Event()
        self.polling_event.set()  # Allow the polling thread to run
        self.polling_thread = threading.Thread(target=self.poll_serial_data)
        self.polling_thread.daemon = True  # Ensure the thread exits when the program exits
        self.polling_thread.start()

    def poll_serial_data(self):
        while True:
            self.polling_event.wait()

            # Check if there are enough bytes available for a complete message
            if self.ser.in_waiting >= 10:
                data = self.ser.read(10)

                # Validate the start byte
                if data[0] != 0xAE:
                    self.get_logger().warning(f"Invalid start byte: {data[0]}")
                    continue

                # Validate the checksum
                checksum = sum(data[:-1]) % 256
                if checksum != data[-1]:
                    self.get_logger().warning("Checksum does not match. Discarding message.")
                    continue

                # Parse the ticks values
                ticks1 = (data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4]
                ticks2 = (data[5] << 24) | (data[6] << 16) | (data[7] << 8) | data[8]

                # Handle signed integers
                if ticks1 & (1 << 31):  # If the most significant bit is set
                    ticks1 -= (1 << 32)  # Convert to signed 32-bit integer
                if ticks2 & (1 << 31):
                    ticks2 -= (1 << 32)

                # Publish the ticks as an Int32MultiArray
                msg = Int32MultiArray()
                msg.data = [ticks1, ticks2]
                self.tick_publisher.publish(msg)

        
    def cmd_cartesian_callback(self, msg):
        # Prevent the polling thread from running while sending a command
        self.polling_event.clear()

        message = self.construct_message(0x01, msg.axis, msg.position)
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
        
        # Allow the polling thread to run again
        self.polling_event.set()

    def construct_message(self, message_type, axis, position):
        if message_type not in [0x01, 0x02, 0x03, 0x04]:
            raise ValueError("Invalid message type.")

        if axis not in [0, 1, 2]:
            raise ValueError("Axis must be 0, 1, or 2.")

        if not (0 <= position <= 450):
            raise ValueError("Position must be between 0 and 450.")
        
        pos_high = (position >> 8) & 0xFF
        pos_low = position & 0xFF
        checksum = (message_type + axis + pos_high + pos_low) % 256
        message = bytes([message_type, axis, pos_high, pos_low, checksum])
        
        return message

    def parse_message(self, message):
        if len(message) != 5:
            raise ValueError("Message must be 5 bytes long.")
        
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
                        message_type, resp_axis, resp_position = self.parse_message(message)
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
                        message_type, resp_axis, resp_position = self.parse_message(message)
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
        self.polling_event.clear()
        self.serial_thread.join()
        self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UartNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

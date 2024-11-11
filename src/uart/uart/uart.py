import rclpy
from rclpy.node import Node
import serial
import time

from custom_msgs.msg import CartesianMsg

class UartNode(Node):
    def __init__(self):
        super().__init__('uart_node')

        self.subscriber = self.create_subscription(
            CartesianMsg,
            "/cmd_cartesian",
            self.cmd_cartesian_callback()
            10
        )

        self.publisher = self.create_publisher(
            CartesianMsg,
            "/cartesian_state",
            10
        )

        self.ser = serial.Serial('/dev/ttyAMA0', baudrate=9600, timeout=0.1)

    def cmd_cartesian_callback(self, msg):
        message = self.construct_message(0x01, msg.axis, msg.position)
        self.ser.write(message)
        self.get_logger().info(f"Sent Command: Axis {msg.axis}, Position {msg.position}")

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

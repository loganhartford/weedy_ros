import serial
import time

from utils.exceptions import UARTError

class UART:
    def __init__(self):
         self.ser = serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=0.1)

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

        return ticks1, ticks2, stamp
    
    def send_command(self, axis, position):
        pos_high = (position >> 8) & 0xFF
        pos_low = position & 0xFF
        checksum = (0x01 + axis + pos_high + pos_low) % 256
        message = bytes([0x01, axis, pos_high, pos_low, checksum])

        if not self.wait_for_acknowledgment():
            raise UARTError("Timeout waiting for acknowledgment.")
        
        data_received, resp_axis, resp_position = self.wait_for_data_message()
        if not data_received:
            raise UARTError("Timeout waiting for response.")
        
        return True

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
                        if message[0] == 0x03: # ACK
                            buffer = buffer[5:]
                            return True
                        else:
                            buffer = buffer[5:]
                            raise UARTError("Incorrect message type for ACK.")
                    except ValueError as e:
                        buffer = buffer[1:]  # Remove one byte and retry
            else:
                time.sleep(0.1)

        return False
    
    def wait_for_data_message(self):
        start_time = time.time()
        buffer = bytearray()

        while True: # For now, just wait indefinitely
            data = self.ser.read(self.ser.in_waiting or 1)
            if data:
                buffer.extend(data)
                # Try to parse complete messages
                while len(buffer) >= 5:
                    message = buffer[:5]
                    try:
                        if message[0] == 0x02:  # Data message type
                            buffer = buffer[5:]  # Remove processed message
                            return True
                        else:
                            buffer = buffer[5:]
                            raise UARTError("Incorrect message type for response.")
                    except ValueError as e:
                        buffer = buffer[1:]  # Remove one byte and retry
            else:
                time.sleep(0.1)

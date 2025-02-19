from rclpy.clock import Clock
import serial
import time
from utils.exceptions import UARTError
from utils.nucleo_gpio import NucleoGPIO

class UART:
    """
    Handles serial communication with the robot's UART device (Nucleo).
    
    This class provides methods to:
      - Request and parse encoder ticks.
      - Read the battery voltage.
      - Send manual control commands.
      - Send motion commands and wait for acknowledgments.
    """
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=0.1)
        self.clock = Clock()
        self.ticks_timeout = 0.01  # seconds
        self.ack_timeout = 0.1     # seconds
        self.num_tick_timeouts = 0

        # Command bytes
        self.ticks_byte = 0xAE
        self.battery_byte = 0x11
        self.left_byte = 0x7E
        self.right_byte = 0x3F
        self.up_byte = 0x5A
        self.down_byte = 0x9A
        self.drill_byte = 0x0F
        self.stop_byte = 0x07

        self.nucleo_gpio = NucleoGPIO()

    def get_ticks(self):
        """
        Requests encoder ticks and returns (ticks1, ticks2, timestamp).
        
        Expects a 6-byte response:
          - Start byte, 2 bytes for left ticks, 2 bytes for right ticks, checksum.
        """
        # Clear any leftover data
        if self.ser.in_waiting:
            self.ser.reset_input_buffer()

        self.ser.write(bytes([self.ticks_byte]))

        buffer = bytearray()
        start_time = time.time()
        while len(buffer) < 6:
            if (time.time() - start_time) > self.ticks_timeout:
                self.num_tick_timeouts += 1
                if self.num_tick_timeouts > 3:
                    self.nucleo_gpio.toggle_reset()
                    raise UARTError("Too many timeouts waiting for ticks, resetting Nucleo.")
            data = self.ser.read(self.ser.in_waiting or 1)
            if data:
                buffer.extend(data)
        stamp = self.clock.now()
        self.num_tick_timeouts = 0

        # Validate response: start byte and checksum
        if buffer[0] != self.ticks_byte:
            raise UARTError("Invalid start byte in ticks reply.")

        if sum(buffer[:-1]) % 256 != buffer[-1]:
            raise UARTError("Invalid tick checksum.")

        # Parse tick values (handle signed 16-bit integers)
        ticks1 = (buffer[1] << 8) | buffer[2]
        ticks2 = (buffer[3] << 8) | buffer[4]
        if ticks1 & (1 << 15):
            ticks1 -= (1 << 16)
        if ticks2 & (1 << 15):
            ticks2 -= (1 << 16)

        return ticks1, ticks2, stamp

    def manual_control(self, direction):
        """
        Sends a manual control command.
        
        Valid directions: 'left', 'right', 'up', 'down', 'drill', 'stop'.
        """
        command_bytes = {
            "left": self.left_byte,
            "right": self.right_byte,
            "up": self.up_byte,
            "down": self.down_byte,
            "drill": self.drill_byte,
            "stop": self.stop_byte
        }
        if direction not in command_bytes:
            raise UARTError("Invalid manual control command.")
        
        self.ser.write(bytes([command_bytes[direction]]))

    def get_battery_voltage(self):
        """
        Requests and returns the battery voltage as a float.
        
        Expects a 4-byte response:
          - Start byte, integer part, decimal part, checksum.
        """
        if self.ser.in_waiting:
            self.ser.reset_input_buffer()

        self.ser.write(bytes([self.battery_byte]))

        buffer = bytearray()
        start_time = time.time()
        while len(buffer) < 4:
            if (time.time() - start_time) > self.ack_timeout:
                raise UARTError("Timeout waiting for battery reply.")
            data = self.ser.read(self.ser.in_waiting or 1)
            if data:
                buffer.extend(data)

        if buffer[0] != self.battery_byte:
            raise UARTError("Invalid start byte in battery reply.")

        if sum(buffer[:-1]) % 256 != buffer[-1]:
            raise UARTError("Invalid battery checksum.")

        int_part = buffer[1]
        decimal_part = buffer[2]
        voltage = int_part + (decimal_part / 100)
        return voltage

    def send_command(self, axis, position, wait=True):
        """
        Sends a command with the given axis and position.
        
        Constructs a message and waits for an acknowledgment and data response.
        Returns the data response if successful.
        """
        position = int(position)
        pos_high = (position >> 8) & 0xFF
        pos_low = position & 0xFF
        checksum = (0x01 + axis + pos_high + pos_low) % 256
        message = bytes([0x01, axis, pos_high, pos_low, checksum])

        self.ser.write(message)

        if not self.wait_for_acknowledgment():
            raise UARTError("Timeout waiting for acknowledgment.")

        if wait:
            if not self.wait_for_data_message():
                raise UARTError("Timeout waiting for response.")

        return True

    def wait_for_acknowledgment(self):
        """
        Waits for an acknowledgment message (5 bytes, starting with 0x03).
        Returns True if an ACK is received before timeout.
        """
        start_time = time.time()
        buffer = bytearray()
        while (time.time() - start_time) < self.ack_timeout:
            data = self.ser.read(self.ser.in_waiting or 1)
            if data:
                buffer.extend(data)
                while len(buffer) >= 5:
                    message = buffer[:5]
                    if message[0] == 0x03:  # ACK message type
                        return True
                    buffer = buffer[5:]
            else:
                time.sleep(0.1)
        return False

    def wait_for_data_message(self):
        """
        Waits for a data message (5 bytes, starting with 0x02).
        Returns True once a valid data message is received.
        """
        start_time = time.time()
        buffer = bytearray()
        while True:
            data = self.ser.read(self.ser.in_waiting or 1)
            if data:
                buffer.extend(data)
                while len(buffer) >= 5:
                    message = buffer[:5]
                    if message[0] == 0x02:  # Data message type
                        return True
                    buffer = buffer[5:]
            else:
                time.sleep(0.1)

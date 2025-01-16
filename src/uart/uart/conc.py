import serial
import time

ser = serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=0.1)

while True:
    time.sleep(0.1)
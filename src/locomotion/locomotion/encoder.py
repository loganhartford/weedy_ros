from rpi_hardware_pwm import HardwarePWM
import Encoder
import time

pwm = HardwarePWM(pwm_channel=0, hz=20000, chip=2)
pwm.start(0)


enc = Encoder.Encoder(23, 24)
enc.read()

current_time = time.time()
pwm.change_duty_cycle(10)

try:
    while True:
        if time.time() - current_time >= 0.5:
            current_time = time.time()
            print(f"Encoder value: {enc.read()}")
except KeyboardInterrupt:
    pwm.stop()
    print(f"Final encoder value: {enc.read()}")
    print("Program interrupted by user.")
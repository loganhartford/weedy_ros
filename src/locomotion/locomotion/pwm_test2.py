from rpi_hardware_pwm import HardwarePWM
import time

pwm = HardwarePWM(pwm_channel=0, hz=20000, chip=2)
pwm.start(0)

try:
    while True:
        pwm.change_duty_cycle(10)

except KeyboardInterrupt:
    pass
finally:
    pwm.stop()
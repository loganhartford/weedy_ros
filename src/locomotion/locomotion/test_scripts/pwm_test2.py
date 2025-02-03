from rpi_hardware_pwm import HardwarePWM
import time

pwm1 = HardwarePWM(pwm_channel=0, hz=20000, chip=2)
pwm2 = HardwarePWM(pwm_channel=1, hz=20000, chip=2)
pwm1.start(0)
pwm2.start(0)

# pwm.change_duty_cycle(10)
# time.sleep(1)

duty = 8

try:
    while True:
        pwm1.change_duty_cycle(duty)
        pwm2.change_duty_cycle(duty)

except KeyboardInterrupt:
    pass
finally:
    pwm1.stop()
    pwm2.stop()
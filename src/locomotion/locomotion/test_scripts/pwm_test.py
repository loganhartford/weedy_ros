from rpi_hardware_pwm import HardwarePWM
import time

pwm = HardwarePWM(pwm_channel=0, hz=20000, chip=2)
pwm.start(0)

try:
    while True:
        # Increase duty cycle from 0 to 100
        for duty_cycle in range(0, 101):
            pwm.change_duty_cycle(duty_cycle)
            time.sleep(0.05)  # Adjust the sleep time as needed

        # Decrease duty cycle from 100 to 0
        for duty_cycle in range(100, -1, -1):
            pwm.change_duty_cycle(duty_cycle)
            time.sleep(0.05)  # Adjust the sleep time as needed

except KeyboardInterrupt:
    pass
finally:
    pwm.stop()
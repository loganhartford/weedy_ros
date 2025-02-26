#!/mnt/shared/weedy_ros/src/locomotion/locomotion/pwm_venv/bin/python3

from rpi_hardware_pwm import HardwarePWM
import lgpio

import utils.robot_params as rp

# Constants for motor direction control
FORWARD = 0
BACKWARD = 1

class MotorController:
    def __init__(self):
        self.left_motor = HardwarePWM(pwm_channel=1, hz=20000, chip=2)
        self.right_motor = HardwarePWM(pwm_channel=0, hz=20000, chip=2)
        self.left_motor.start(0)
        self.right_motor.start(0)

        self.left_motordir = 5
        self.right_motordir = 6
        self.chip = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.chip, self.left_motordir)
        lgpio.gpio_claim_output(self.chip, self.right_motordir)

        if rp.log:
            self.log_file = "/mnt/shared/weedy_ros/src/locomotion/locomotion/outputs/duty_rp.log.csv"
            with open(self.log_file, "w") as file:
                file.write("duty_left,duty_right\n")

    def set_velocity(self, linear_x, angular_z, closed_loop=True):
        linear_x = max(-rp.max_linear_speed, min(linear_x, rp.max_linear_speed))
        angular_z = max(-rp.max_angular_speed, min(angular_z, rp.max_angular_speed))

        left_wheel_velocity = linear_x + (angular_z * rp.wheel_base / 2)
        right_wheel_velocity = linear_x - (angular_z * rp.wheel_base / 2)

        left_wheel_ang_vel = left_wheel_velocity / rp.wheel_radius
        right_wheel_ang_vel = right_wheel_velocity / rp.wheel_radius

        # Normalize to a duty cycle
        left_duty = min(max(abs(left_wheel_ang_vel) / rp.rated_speed * 100, 0), 100)
        right_duty = min(max(abs(right_wheel_ang_vel) / rp.rated_speed * 100, 0), 100)

        # Set motor direction based on velocity sign.
        lgpio.gpio_write(self.chip, self.left_motordir, BACKWARD if left_wheel_velocity >= 0 else FORWARD)
        lgpio.gpio_write(self.chip, self.right_motordir, FORWARD if right_wheel_velocity >= 0 else BACKWARD)

        # To drive down the last bit of error without stopping from stiction
        if closed_loop:
            if left_duty > 0:
                left_duty = max(left_duty, rp.min_duty_cycle)
            if right_duty > 0:
                right_duty = max(right_duty, rp.min_duty_cycle)

        if rp.log:
            with open(self.log_file, "a") as file:
                file.write(f"{left_duty},{right_duty}\n")

        self.left_motor.change_duty_cycle(left_duty)
        self.right_motor.change_duty_cycle(right_duty * rp.motor_comp_factor)

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()
        lgpio.gpiochip_close(self.chip)

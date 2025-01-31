#!/mnt/shared/weedy_ros/src/locomotion/locomotion/pwm_venv/bin/python3

from rpi_hardware_pwm import HardwarePWM
import lgpio

from utils.robot_params import wheel_radius, wheel_base, rated_speed, min_duty_cycle

# TODO: validate on robot
FORWARD = 0
BACKWARD = 1

class MotorController:
    def __init__(self):
        self.motor1 = HardwarePWM(pwm_channel=1, hz=20000, chip=2)
        self.motor2 = HardwarePWM(pwm_channel=0, hz=20000, chip=2)
        self.motor1.start(0)
        self.motor2.start(0)

        self.motor1dir = 5
        self.motor2dir = 6
        self.chip = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.chip, self.motor1dir)
        lgpio.gpio_claim_output(self.chip, self.motor2dir)

        self.wheel_radius = wheel_radius    # meters -> 5in
        self.wheel_base = wheel_base        # meters TODO: update this value
        self.rated_speed = rated_speed      # rad/s -> 230 RPM

    def set_velocity(self, linear_x, angular_z):
        # Calculate wheel velocities in m/s
        left_wheel_velocity = linear_x - angular_z * self.wheel_base / 2
        right_wheel_velocity = linear_x + angular_z * self.wheel_base / 2

        # Convert wheel velocities to angular velocities (rad/s)
        left_wheel_angular_velocity = left_wheel_velocity / self.wheel_radius
        right_wheel_angular_velocity = right_wheel_velocity / self.wheel_radius

        # Normalize angular velocities to the motor's rated speed
        left_duty_cycle = min(max(abs(left_wheel_angular_velocity) / self.rated_speed * 100, 0), 100)
        right_duty_cycle = min(max(abs(right_wheel_angular_velocity) / self.rated_speed * 100, 0), 100)

        # Set direction pins based on the sign of the velocity
        lgpio.gpio_write(self.chip, self.motor1dir, BACKWARD if left_wheel_velocity >= 0 else FORWARD)
        lgpio.gpio_write(self.chip, self.motor2dir, FORWARD if right_wheel_velocity >= 0 else BACKWARD)

        # Motors won't move unless duty cycle is greater than 3
        if left_duty_cycle > 0:
            left_duty_cycle = max(left_duty_cycle, min_duty_cycle)
        
        if right_duty_cycle > 0:
            right_duty_cycle = max(right_duty_cycle, min_duty_cycle)

        self.motor1.change_duty_cycle(left_duty_cycle)
        self.motor2.change_duty_cycle(right_duty_cycle)

    def stop(self):
        self.motor1.stop()
        self.motor2.stop()
        lgpio.gpiochip_close(self.chip)



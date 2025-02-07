#!/mnt/shared/weedy_ros/src/locomotion/locomotion/pwm_venv/bin/python3

from rpi_hardware_pwm import HardwarePWM
import lgpio

from utils.robot_params import wheel_radius, wheel_base, rated_speed, min_duty_cycle, motor_comp_factor

# TODO: validate on robot
FORWARD = 0
BACKWARD = 1
LOG = True

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

        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.rated_speed = rated_speed

        # log duty cycles to a csv
        if LOG:
            self.log_file = "/mnt/shared/weedy_ros/src/locomotion/locomotion/outputs/duty_log.csv"
            with open(self.log_file, "w") as file:
                file.write("duty_left,duty_right\n") 

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
        lgpio.gpio_write(self.chip, self.left_motordir, BACKWARD if left_wheel_velocity >= 0 else FORWARD)
        lgpio.gpio_write(self.chip, self.right_motordir, FORWARD if right_wheel_velocity >= 0 else BACKWARD)

        # Motors won't move unless duty cycle is greater than 3
        if left_duty_cycle > 0:
            left_duty_cycle = max(left_duty_cycle, min_duty_cycle)
        
        if right_duty_cycle > 0:
            right_duty_cycle = max(right_duty_cycle, min_duty_cycle)

        if LOG:
            with open(self.log_file, "a") as file:
                file.write(f"{left_duty_cycle},{right_duty_cycle}\n")

        self.left_motor.change_duty_cycle(left_duty_cycle)
        self.right_motor.change_duty_cycle(right_duty_cycle*motor_comp_factor)

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()
        lgpio.gpiochip_close(self.chip)



#!/mnt/shared/weedy_ros/src/locomotion/locomotion/pwm_venv/bin/python3

from rpi_hardware_pwm import HardwarePWM
import lgpio
from utils.robot_params import wheel_radius, wheel_base, rated_speed, min_duty_cycle, motor_comp_factor

# Constants for motor direction control
FORWARD = 0
BACKWARD = 1
LOG = True

class MotorController:
    """
    Controls the front-wheel motors of a four-wheeled robot using PWM and GPIO.
    The front wheels are driven; the back wheels are passive.
    """
    def __init__(self):
        # Initialize PWM channels for left and right motors.
        self.left_motor = HardwarePWM(pwm_channel=1, hz=20000, chip=2)
        self.right_motor = HardwarePWM(pwm_channel=0, hz=20000, chip=2)
        self.left_motor.start(0)
        self.right_motor.start(0)

        # Initialize GPIO for motor direction control.
        self.left_motordir = 5
        self.right_motordir = 6
        self.chip = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.chip, self.left_motordir)
        lgpio.gpio_claim_output(self.chip, self.right_motordir)

        # Store robot parameters.
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.rated_speed = rated_speed

        # Initialize duty cycle logging if enabled.
        if LOG:
            self.log_file = "/mnt/shared/weedy_ros/src/locomotion/locomotion/outputs/duty_log.csv"
            with open(self.log_file, "w") as file:
                file.write("duty_left,duty_right\n")

    def set_velocity(self, linear_x, angular_z):
        """
        Sets motor speeds based on desired linear (m/s) and angular (rad/s) velocities.
        Calculates individual wheel speeds, converts them to PWM duty cycles,
        and sets the corresponding GPIO direction pins.
        """
        # Calculate wheel velocities (m/s)
        left_wheel_velocity = linear_x + (angular_z * self.wheel_base / 2)
        right_wheel_velocity = linear_x - (angular_z * self.wheel_base / 2)

        # Convert wheel velocities to angular velocities (rad/s)
        left_wheel_ang_vel = left_wheel_velocity / self.wheel_radius
        right_wheel_ang_vel = right_wheel_velocity / self.wheel_radius

        # Normalize to a duty cycle percentage based on rated speed.
        left_duty = min(max(abs(left_wheel_ang_vel) / self.rated_speed * 100, 0), 100)
        right_duty = min(max(abs(right_wheel_ang_vel) / self.rated_speed * 100, 0), 100)

        # Set motor direction based on velocity sign.
        lgpio.gpio_write(self.chip, self.left_motordir, BACKWARD if left_wheel_velocity >= 0 else FORWARD)
        lgpio.gpio_write(self.chip, self.right_motordir, FORWARD if right_wheel_velocity >= 0 else BACKWARD)

        # Ensure the duty cycle is above the minimum threshold if nonzero.
        if left_duty > 0:
            left_duty = max(left_duty, min_duty_cycle)
        if right_duty > 0:
            right_duty = max(right_duty, min_duty_cycle)

        # Log the duty cycles if logging is enabled.
        if LOG:
            with open(self.log_file, "a") as file:
                file.write(f"{left_duty},{right_duty}\n")

        # Set the PWM duty cycles (apply compensation factor to the right motor).
        self.left_motor.change_duty_cycle(left_duty)
        self.right_motor.change_duty_cycle(right_duty * motor_comp_factor)

    def stop(self):
        """
        Stops both motors and releases the GPIO chip.
        """
        self.left_motor.stop()
        self.right_motor.stop()
        lgpio.gpiochip_close(self.chip)

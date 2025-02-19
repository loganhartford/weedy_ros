from rclpy.time import Time
from utils.robot_params import log

class PID_ctrl:
    """
    A PID controller that maintains a fixed-length history of error measurements.
    It computes the control output based on proportional, integral, and derivative terms.
    Optionally logs data to a CSV file.
    """
    def __init__(self, kp, kd, ki, history_length=10,
                 log_file="/mnt/shared/weedy_ros/src/locomotion/locomotion/outputs/pid_log.csv"):
        # Controller gains
        self.kp = kp
        self.kd = kd
        self.ki = ki

        # History of (error, timestamp) tuples
        self.history_length = history_length
        self.history = []

        # Logging setup
        if log:
            self.log_file = log_file
            with open(self.log_file, "w") as file:
                file.write("Timestamp,Error,P_Term,I_Term,D_Term,Control_Output\n")

    def clear_history(self):
        """Clear the stored error history."""
        self.history = []

    def update(self, error_stamp_tuple):
        """
        Update the PID controller with a new error measurement.
        
        Args:
            error_stamp_tuple (tuple): A tuple (error, stamp) where 'error' is the current error
                                       and 'stamp' is a ROS Time message.
        
        Returns:
            float: The computed control output.
        """
        error, stamp = error_stamp_tuple
        self.history.append(error_stamp_tuple)

        # Maintain fixed-length history
        if len(self.history) > self.history_length:
            self.history.pop(0)

        # Until the history is full, use only the proportional term.
        if len(self.history) < self.history_length:
            return self.kp * error

        # Compute average dt and error derivative over the history.
        total_dt = 0.0
        total_derivative = 0.0
        # Use (N - 1) intervals for N measurements.
        for i in range(1, self.history_length):
            t_prev = Time.from_msg(self.history[i - 1][1])
            t_curr = Time.from_msg(self.history[i][1])
            dt = max((t_curr.nanoseconds - t_prev.nanoseconds) / 1e9, 1e-6)
            total_dt += dt
            total_derivative += (self.history[i][0] - self.history[i - 1][0]) / dt

        avg_dt = total_dt / (self.history_length - 1)
        avg_derivative = total_derivative / (self.history_length - 1)

        # Compute the integral term as the sum of errors times the average dt.
        error_sum = sum(e for e, _ in self.history)
        error_integral = error_sum * avg_dt

        # Compute PID terms.
        p_term = self.kp * error
        i_term = self.ki * error_integral
        d_term = self.kd * avg_derivative

        control_output = p_term + i_term + d_term

        if log:
            self.log_data(stamp, error, p_term, i_term, d_term, control_output)

        return control_output

    def log_data(self, stamp, error, p_term, i_term, d_term, control_output):
        """
        Log the PID terms and output to the CSV file.
        
        Args:
            stamp: A ROS Time message.
            error (float): The current error.
            p_term (float): The proportional term.
            i_term (float): The integral term.
            d_term (float): The derivative term.
            control_output (float): The total control output.
        """
        timestamp = Time.from_msg(stamp).nanoseconds / 1e9  # Convert to seconds
        with open(self.log_file, "a") as file:
            file.write(f"{timestamp},{error},{p_term},{i_term},{d_term},{control_output}\n")

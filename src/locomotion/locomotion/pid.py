from rclpy.time import Time

LOG = False

class PID_ctrl:
    
    def __init__(self, kp, kd, ki, history_length=10, log_file="outputs/pid_log.csv"):
        # Data for the controller
        self.history_length = history_length
        self.history = []

        # Controller gains
        self.kp = kp
        self.kd = kd
        self.ki = ki

        # Logging setup
        if LOG:
            self.log_file = log_file
            with open(self.log_file, "w") as file:
                file.write("Timestamp,Error,P_Term,I_Term,D_Term,Control_Output\n")  # CSV Header

    def clear_history(self):
        self.history = []
    
    def update(self, stamped_error):
        latest_error = stamped_error[0]
        stamp = stamped_error[1]
        
        self.history.append(stamped_error)        
        
        if len(self.history) > self.history_length:
            self.history.pop(0)
        
        if len(self.history) != self.history_length:
            return self.kp * latest_error
        
        # Compute the error derivative
        dt_avg = 0
        error_dot = 0
        
        for i in range(1, len(self.history)):
            t0 = Time.from_msg(self.history[i-1][1])
            t1 = Time.from_msg(self.history[i][1])
            
            dt = max((t1.nanoseconds - t0.nanoseconds) / 1e9, 1e-6)
            dt_avg += dt   

            error_dot += (self.history[i][0] - self.history[i - 1][0]) / dt
            
        error_dot /= len(self.history)
        dt_avg /= len(self.history)
        
        # Compute the error integral
        error_sum = sum(hist[0] for hist in self.history)
        error_int = error_sum * dt_avg

        # Compute PID terms
        p_term = self.kp * latest_error
        i_term = self.ki * error_int
        d_term = self.kd * error_dot

        # Compute total control output
        control_output = p_term + i_term + d_term

        if LOG:
            self.log_data(stamp, latest_error, p_term, i_term, d_term, control_output)

        return control_output

    def log_data(self, stamp, error, p_term, i_term, d_term, control_output):
        timestamp = Time.from_msg(stamp).nanoseconds / 1e9  # Convert to seconds
        with open(self.log_file, "a") as file:
            file.write(f"{timestamp},{error},{p_term},{i_term},{d_term},{control_output}\n")

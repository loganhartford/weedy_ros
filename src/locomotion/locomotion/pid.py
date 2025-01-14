from rclpy.time import Time

class PID_ctrl:
    
    def __init__(self, type_, kp=1.2,kv=0.8,ki=0.2, history_length=3):
        
        # Data for the controller
        self.history_length = history_length
        self.history = []

        # Controller gains
        self.kp = kp
        self.kv = kv
        self.ki = ki

    def clear_history(self):
        self.history = []
    
    def update(self, stamped_error):
        latest_error = stamped_error[0]
        stamp = stamped_error[1]
        
        self.history.append(stamped_error)        
        
        if (len(self.history) > self.history_length):
            self.history.pop(0)
        
        # If insufficient data points, use only the proportional gain
        if (len(self.history) != self.history_length):
            return self.kp * latest_error
        
        # Compute the error derivative
        dt_avg=0
        error_dot=0
        
        for i in range(1, len(self.history)):
            
            t0=Time.from_msg(self.history[i-1][1])
            t1=Time.from_msg(self.history[i][1])
            
            dt=(t1.nanoseconds - t0.nanoseconds) / 1e9
            
            dt_avg+=dt

            error_dot += (self.history[i][0] - self.history[i - 1][0]) / dt
            
        error_dot/=len(self.history)
        dt_avg/=len(self.history)
        
        # Compute the error integral
        sum_=0
        for hist in self.history:
            # DONE Part 5: Gather the integration
            sum_ += hist[0]
            pass
        
        error_int=sum_*dt_avg
        
        return (self.kp * latest_error) + (self.ki * error_int) + (self.kv * error_dot)

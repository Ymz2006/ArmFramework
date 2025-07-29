import time

class PID():
    def __init__(self, target, kp, ki, kd, deadzone=0.0):
        self.target = target
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.deadzone = deadzone

        self.last_error = 0.0
        self.integral = 0.0
        self.last_update = time.time()
    
    def update(self, measurement):
        curr_time = time.time()
        elapsed_time = curr_time - self.last_update
        self.last_update = curr_time

        error = self.target - measurement
        
        if abs(error) < self.deadzone:
            error = 0.0

        self.integral += error * elapsed_time
        derivative = (error - self.last_error) / elapsed_time if elapsed_time > 0 else 0.0
        self.last_error = error

        p_term = self.kp * error
        i_term = self.ki * self.integral
        d_term = self.kd * derivative

        return p_term + i_term + d_term

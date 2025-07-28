import time
class PID():
    def __init__(self, target, kp, ki, kd, deadzone):
        self.target = target
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0
        self.integral = 0
        self.last_update = time.time()
        self.deadzone = deadzone
        return
    
    
    def update(self, measurment):
        error = self.target-measurment
        self.integral += error

        curr_time = time.time()
        elapsed_time = curr_time - self.last_update

        self.last_update = curr_time
        
        porportional = self.kp * error
        integral = self.ki * self.integral
        derivative = self.kd * (self.last_error - measurment)/elapsed_time 

   
        
        return porportional+integral+derivative
    
    


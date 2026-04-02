class robot_PID:

    Kp: float
    Ki: float
    Kd: float

    def __init__(self, Kp=0.7, Ki=0.0, Kd=0.04):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = 0
        self.previous_error = 0
        self.integral = 0

    def compute(self, process_variable, dt=0.1):
        # Calculate error
        error = self.setpoint - process_variable
        
        # Proportional term
        P_out = self.Kp * error
        
        # Integral term
        self.integral += error * dt
        I_out = self.Ki * self.integral
        
        # Derivative term
        derivative = (error - self.previous_error) / dt
        D_out = self.Kd * derivative
        
        # Compute total output
        output = P_out + I_out + D_out
        
        # Update previous error
        self.previous_error = error
        
        return output
    
    def update_setpoint(self, setpoint):
        self.setpoint = setpoint
        self.reset_integral()
    
    def reset_integral(self):
        self.integral = 0
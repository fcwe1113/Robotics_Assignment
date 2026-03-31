class robot_PID:

    Kp: float
    # Ki: float
    Kd: float

    def __init__(self, Kp=0.001, Ki=0.001, Kd=0.001):
        self.Kp = Kp
        # self.Ki = Ki
        self.Kd = Kd
        self.setpoint = 0
        self.previous_error = 0
        # self.integral = 0

    def compute(self, process_variable, dt=0.1):
        # Calculate error
        error = self.setpoint - process_variable
        
        # Proportional term
        P_out = self.Kp * error
        
        # Integral term
        # self.integral += error * dt
        # I_out = self.Ki * self.integral
        
        # Derivative term
        derivative = (error - self.previous_error) / dt
        D_out = self.Kd * derivative
        
        # Compute total output
        output = P_out + 0 + D_out
        
        # Update previous error
        self.previous_error = error
        
        return output
    
    def update_setpoint(self, setpoint):
        self.setpoint = setpoint
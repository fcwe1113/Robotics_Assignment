class robot_PID:

    """
    Robot PID controller class

    :cvar Kp: Proportional gain
    :cvar Ki: Integral gain
    :cvar Kd: Derivative gain
    :cvar setpoint: The target value
    :cvar previous_error: Previous error from last tick
    :cvar integral: Accumulated integral value
    """

    Kp: float
    Ki: float
    Kd: float

    def __init__(self, Kp=0.7, Ki=0.0, Kd=0.04):
        """
        Constructor for Robot PID class

        :param float Kp: Proportional gain
        :param float Ki: Integral gain
        :param float Kd: Derivative gain
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = 0
        self.previous_error = 0
        self.integral = 0

    def compute(self, process_variable, dt=0.1):
        """
        Compute the PID controller output

        :param process_variable: Current state
        :param dt: Defines the Integral value effect
        :returns: PID controller output
        :rtype: float
        """
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
        """
        Update the setpoint value

        :param float setpoint: Target setpoint
        :rtype: None
        """
        self.setpoint = setpoint
        self.reset_integral()
    
    def reset_integral(self):
        """
        Reset the integral value to 0

        :rtype: None
        """
        self.integral = 0
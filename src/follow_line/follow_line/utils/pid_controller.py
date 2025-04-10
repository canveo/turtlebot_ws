class PIDController:
    """Controlador PID básico."""
    def __init__(self, kp, ki, kd, max_output=float('inf'), min_output=float('-inf')):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_error = 0.0 # antiwindup

    def compute(self, error, delta_time):
        """Calcula la salida del PID."""
        if delta_time <= 0.0:
            return 0.0
        
        # Proporcional
        p = self.kp * error

        # Integral
        self.integral += error * delta_time
        i = self.ki * self.integral

        # Derivativo
        d = self.kd * (error - self.prev_error) / delta_time
        self.prev_error = error

        # Salida limitada
        output = p + i + d
        output = max(self.min_output, min(self.max_output, output))

        if output == self.max_output or output == self.min_output:
            self.integral -= error * delta_time
        return output
      
      
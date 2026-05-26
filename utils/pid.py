"""
Controlador PID simple para lazo cerrado de posición.
"""

import time

class PIDController:
    def __init__(self, kp, ki, kd, output_limit=30):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit

        self.prev_error = 0
        self.integral = 0
        self.last_time = None

    def compute(self, error):
        """Calcula la salida del PID dado el error actual."""
        now = time.time()
        if self.last_time is None:
            dt = 0.05  # Primera iteración, asumir 50ms
        else:
            dt = now - self.last_time
            dt = max(dt, 0.001)  # Evitar dt = 0
        self.last_time = now

        # Proporcional
        P = self.kp * error

        # Integral (con anti-windup)
        self.integral += error * dt
        self.integral = max(-self.output_limit / max(self.ki, 0.001),
                           min(self.integral, self.output_limit / max(self.ki, 0.001)))
        I = self.ki * self.integral

        # Derivativo
        D = self.kd * (error - self.prev_error) / dt
        self.prev_error = error

        # Salida total con saturación
        output = P + I + D
        output = max(-self.output_limit, min(output, self.output_limit))
        return int(output)

    def reset(self):
        self.prev_error = 0
        self.integral = 0
        self.last_time = None

import numpy as np

class PID:
    def __init__(self, Kp = 0., Ki = 0., Kd = 0.):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.bias = 0.

        self.set_point = 0.
        self.integral = 0.
        self.prev_error = 0.
        self.dt = 0.1
        self.prev_dt = 0.1

        self.P_term = 0.
        self.I_term = 0.
        self.D_term =0.

        self.integral_cutoff = None

    def update(self, feedback, dt=None):
        error = self.set_point - feedback
        if dt is not None:
            if dt == 0:
                self.dt = self.prev_dt
            else:
                self.dt = dt
                self.prev_dt = dt

        self.P_term = self.Kp * error + self.bias

        self.integral += error * self.dt
        self.apply_integral_cutoff()
        self.I_term = self.Ki * self.integral

        d_error = error - self.prev_error
        self.D_term = self.Kd * d_error/self.dt

        control_signal = self.P_term + self.I_term + self.D_term

        return control_signal 
    
    def apply_integral_cutoff(self):
        if self.integral_cutoff != None:
            sign = 1.0
            if self.integral < 0.:
                sign = -1.0
            if abs(self.integral) > self.integral_cutoff:
                self.integral = self.integral_cutoff * sign

    def set_PID_gains(self, Kp, Ki, Kd, bias = 0.):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.bias = bias
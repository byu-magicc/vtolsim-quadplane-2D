"""
quadSim
    - Chapter 14.6 assignment for Beard & McLain, PUP, 2012
    - Update history:
        7/1/2021 - RWB
        4/27/2022 - RWB
"""
import numpy as np


class PIDControl:
    def __init__(self,
                 kp: float = 0.0, 
                 ki: float = 0.0, 
                 kd: float = 0.0, 
                 Ts: float = 0.01, 
                 sigma: float = 0.05, 
                 limit: float = 1.0):
        
        #proportional gain
        self.kp = kp
        #integrator gain
        self.ki = ki
        #derivative gain
        self.kd = kd
        #time sampling constant
        self.Ts = Ts
        self.limit = limit
        #the value of the integrator
        self.integrator = 0.0
        #the derivative of the error
        self.error_dot = 0.0
        #the previous error sample
        self.error_delay_1 = 0.0
        self.y_dot = 0.0
        self.y_delay_1 = 0.0
        # gains for differentiator
        self.a1 = (2.0 * sigma - Ts) / (2.0 * sigma + Ts)
        self.a2 = 2.0 / (2.0 * sigma + Ts)


    def update(self, 
               y_ref: float, 
               y: float, 
               reset_flag: bool = False):
        if reset_flag is True:
            self.integrator = 0.0
            self.error_dot = 0.0
            self.error_delay_1 = 0.0
        # compute the error
        error = y_ref - y
        # update the integrator using trapazoidal rule
        self.integrator = self.integrator + (self.Ts/2) * (error + self.error_delay_1)
        # update the differentiator
        self.error_dot = self.a1 * self.error_dot + self.a2 * (error - self.error_delay_1)
        # PID control
        u = self.kp * error \
            + self.ki * self.integrator \
            + self.kd * self.error_dot
        # saturate PID control at limit
        u_sat = self._saturate(u)
        # integral anti-windup
        #   adjust integrator to keep u out of saturation
        if np.abs(self.ki) > 0.0001:
            self.integrator = self.integrator + (self.Ts / self.ki) * (u_sat - u)
        # update the delayed variables
        self.error_delay_1 = error
        return u_sat

    def update_with_ff(self, y_ref, y, y_ref_dot, feedforward, reset_flag=False):
        if reset_flag is True:
            self.integrator = 0.0
            self.y_dot = 0.0
            self.y_delay_1 = 0.0
            self.error_delay_1 = 0.0
        # compute the error
        error = y_ref - y
        # update the integrator using trapazoidal rule
        self.integrator = self.integrator + (self.Ts/2) * (error + self.error_delay_1)
        self.error_delay_1 = error
        # update the differentiator
        self.y_dot = self.a1 * self.y_dot + self.a2 * (y - self.y_delay_1)
        self.y_delay_1 = y
        # PID control
        u = feedforward \
            + self.kp * error \
            + self.ki * self.integrator \
            + self.kd * (y_ref_dot - self.y_dot)
        # saturate PID control at limit
        u_sat = self._saturate(u)
        # integral anti-windup
        #   adjust integrator to keep u out of saturation
        if np.abs(self.ki) > 0.0001:
            self.integrator = self.integrator + (self.Ts / self.ki) * (u_sat - u)
        # update the delayed variables
        return u_sat

    def update_with_rate(self, y_ref, y, ydot, reset_flag=False):
        if reset_flag is True:
            self.integrator = 0.0
            self.error_delay_1 = 0.0
        # compute the error
        error = y_ref - y
        # update the integrator using trapazoidal rule
        self.integrator = self.integrator + (self.Ts/2) * (error + self.error_delay_1)
        # PID control
        u = self.kp * error \
            + self.ki * self.integrator \
            - self.kd * ydot
        # saturate PID control at limit
        u_sat = self._saturate(u)
        # integral anti-windup
        #   adjust integrator to keep u out of saturation
        if np.abs(self.ki) > 0.0001:
            self.integrator = self.integrator + (self.Ts / self.ki) * (u_sat - u)
        self.error_delay_1 = error
        return u_sat

    def _saturate(self, u):
        # saturate u at +- self.limit
        if u >= self.limit:
            u_sat = self.limit
        elif u <= -self.limit:
            u_sat = -self.limit
        else:
            u_sat = u
        return u_sat
    



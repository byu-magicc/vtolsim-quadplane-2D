#implements the feedforward control
import numpy as np
import os, sys

from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))

from tools.dirtyDerivative import dirtyDerivative

#this is feedforward control to cheat at controlling a double integrator system.
class feedForwardControl:
    
    #Arguments:
    #1. kp - proportional gain
    #2. kd - derivative gain
    #3. Ts - time sample period
    #4. Jy - rotational inertia of system around y axis
    #5. sigma - variable for derivation
    def __init__(self,
                 kp: float,
                 kd: float,
                 Ts: float,
                 Jy: float,
                 u_max: float,
                 sigma: float = 0.05):
        
        self.kp = kp
        self.kd = kd
        self.Ts = Ts
        self.Jy = Jy
        self.sigma = sigma
        self.u_max = u_max


        #variables for saving the variables we are taking the dirty derivative of theta_ref, and theta_dot_ref
        self.theta_dot_ref_diff = dirtyDerivative(Ts=self.Ts,
                                                  sigma=self.sigma)
        #same for theta_ddot
        self.theta_ddot_ref_diff = dirtyDerivative(Ts=self.Ts,
                                                   sigma=self.sigma)
        

    #update function.
    #Arguments:
    #1. state: the state we are trying to control
    #2. state_dot: the derivative of that state
    #3. state_ref: the reference value we are attempting to control the state to
    def update(self,
               state: float,
               state_dot: float,
               state_ref: float):
        
        #gets the state ref dot
        state_ref_dot = self.theta_dot_ref_diff.update(state=state_ref)
        #and the second derivative
        state_ref_ddot = self.theta_ddot_ref_diff.update(state=state_ref_dot)


        #gets state tilde, and state_dot tilde
        state_tilde = state_ref - state
        state_dot_tilde = state_ref_dot - state_dot

        #gets the control law
        u = self.Jy * (state_ref_ddot + self.kp*state_tilde + self.kd*state_dot_tilde)

        #creates the u_saturated
        u_sat = saturate(u=u, limit=self.u_max)

        return u_sat


def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u

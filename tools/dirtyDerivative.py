#implements the dirty derivative function
import numpy as np



class dirtyDerivative:

    #arguments:
    #0. Ts: the time period sample for the controller
    #1. sigma: the variable to help with finding the dirty derivative
    #2. state_0: optional initial state to initialize the derivation
    #3. state_dot_0: the optional initial state derivative to initialize it with
    def __init__(self,
                 Ts: float = 0.01,
                 sigma: float = 0.05,
                 state_0: float = 0.0,
                 state_dot_0: float = 0.0):
        
        self.Ts = Ts
        self.sigma = sigma

        #saves the beta variable
        self.beta = (2.0*self.sigma - self.Ts)

        #initializes the state delayed by 1
        self.state_d1 = state_0
        #initializes the dtate derivative
        self.state_dot = state_dot_0


    def update(self,
               state: float):
        
        #updates the state_derivative
        self.state_dot = self.beta * self.state_dot + (1.0 - self.beta) * ((state - self.state_d1) / self.Ts)

        #saves the previous state
        self.state_d1 = state

        return self.state_dot
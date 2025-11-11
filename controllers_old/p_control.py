#simple, good-ol-fashioned proportional control
import numpy as np

class PControl:

    #initialization function
    def __init__(self,
                 kp: float = 0.0,
                 limit: float = 1.0):
        
        #saves the variables
        self.kp = kp
        self.limit = limit


    #defines the update function
    #1. y_ref: the reference or desired state
    #2. y: the actual state
    def update(self,
               y_ref: float,
               y: float)->float:
        
        #compustes the error
        error = y_ref - y

        #creates the control output u
        u = self.kp * error

        #saturates the output u (to not exceed a certain magnitude)
        u_sat = self._saturate(u=u)

        #returns the U saturated
        return u_sat


    #saturates the control so it is not too large in magnitude
    def _saturate(self, 
                  u: float)->float:
        # saturate u at +- self.limit
        if u >= self.limit:
            u_sat = self.limit
        elif u <= -self.limit:
            u_sat = -self.limit
        else:
            u_sat = u
        return u_sat
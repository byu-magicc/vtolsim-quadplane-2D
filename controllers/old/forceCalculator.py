#This file is very similar to the wrench calculator function,
#except for a few important differences. We create a wrapper around
#the wrench calculator to make things work much more easily

from controllers.wrenchCalculation import wrenchCalculator
from tools.old.rotations import theta_to_rotation_2d
from message_types.msg_delta import MsgDelta
from message_types.msg_state import MsgState
from copy import deepcopy

class ForceCalculator:

    def __init__(self):
        
        #instantiates the wrench calculator
        self.wrenchCalculator = wrenchCalculator()
        pass

    
    def getAerodynamicForces(self,
                             currentState: MsgState,
                             theta: float,
                             gamma: float,
                             Va: float):
        
        #creates a copy of the current state to modify
        #otherwise
        modifiableState = deepcopy(currentState)
        

        #changes the modifiable state to have the airspeed and 
        modifiableState.theta = theta

        #creates the dummy delta input of zero
        delta_temp = MsgDelta(elevator=0.0,
                              throttle_front=0.0,
                              throttle_rear=0.0,
                              throttle_thrust=0.0)
        
        #
        
        #calls the wrench calculator forces moments function
        forces_moment = self.wrenchCalculator.forces_moments_achieved(delta=delta_temp,
                                                                      state=modifiableState)
        
        forces = forces_moment[:-1,:]

        
        return forces
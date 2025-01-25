#This file implements the wrench calculator for the low level controller

from message_types.msg_delta import MsgDelta
from message_types.msg_state import MsgState

#This iteration will NOT take into account gravity. This will only take
#into account the aerodynamic and actuator forces on the aircraft, in 2 Dimensions
class wrenchCalculator:

    #creates the initialization function
    def __init__(self):

        #creates a counter
        self.counter = 0

    

    #creates the forces moments and the Jacobian for it
    def forces_moments_derivatives(self, delta: MsgDelta, state: MsgState):
        
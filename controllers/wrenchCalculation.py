

import numpy as np
from message_types.msg_delta import MsgDelta
from message_types.msg_state import MsgState
from tools.rotations import *
import parameters.anaconda_parameters as QP


class wrenchCalculator: 


    #defines the initialization function
    def __init__(self):

        pass

    #creates the forces and moments and the Jacobian for those forces and moments
    def forces_moments_derivatives(self,
                                   delta: MsgDelta,
                                   state: MsgState):
                                   
        
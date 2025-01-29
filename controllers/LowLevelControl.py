#This file implements the force and torque tracking components which we desire.


import os, sys

from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))


import numpy as np

#imports the delta message type
from message_types.msg_delta import MsgDelta
#imports the state message
from message_types.msg_state import MsgState

import parameters.anaconda_parameters as QP


from tools.rotations import *

import scipy.optimize as spo
from copy import copy
from scipy.optimize import minimize

from controllers.wrenchCalculation import wrenchCalculator

from models.quadplane_dynamics import QuadplaneDynamics



#creates the low level control class
class LowLevelControl:


    #creates the init function
    def __init__(self):

        #creates an instance of the wrench calculator class
        self.wrenchCalculator = wrenchCalculator()

        #creates an instance of the state message class
        self.state = MsgState()

        #creates the previous delta solution
        self.delta_previous = np.array([[]])


    
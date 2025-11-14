import os, sys

from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))


import numpy as np
from controllers.feedforwardControl import feedForwardControl
from message_types.msg_delta import MsgDelta
from message_types.msg_state import MsgState
import parameters.anaconda_parameters as CONDA
from tools.rotations import *

import scipy.optimize as spo
from copy import copy
from scipy.optimize import minimize

from controllers.wrenchCalculation import wrenchCalculator
from models.quadplane_dynamics import QuadplaneDynamics
import parameters.simulation_parameters as SIM
import parameters.control_allocation_parameters as CAP




#creates the low level control class
class LowLevelControl:

    #creates the init function
    def __init__(self,
                 Ts: float = SIM.ts_simulation,
                 pitchControl_riseTime: float = CAP.pitchControl_riseTime,
                 pitchControl_zeta: float = CAP.pitchControl_zeta):
        

        #from this, we get the natural frequency
        pitchControl_naturalFrequency = np.pi / (2.0*pitchControl_riseTime*np.sqrt(1.0 - (pitchControl_zeta**2)))
        

        #creates the three variables for the main PD transfer function here
        b0 = CONDA.Jy
        a1 = 0.0
        a0 = 0.0

        #gets the kp and the kd variables
        kp_pitch = (pitchControl_naturalFrequency**2 - a0)/(b0)
        kd_pitch = (2*pitchControl_zeta*pitchControl_naturalFrequency - a1)/(b0)

        #creates the pitch controller
        self.pitchController = feedForwardControl(kp=kp_pitch,
                                                  kd=kd_pitch,
                                                  Ts=SIM.ts_simulation,
                                                  Jy=CONDA.Jy,
                                                  u_max=CAP.tau_max,
                                                  sigma=0.05)


        #saves the time seperation
        self.Ts = Ts

        #creates an instance of the wrench calculator class
        self.wrenchCalculator = wrenchCalculator()

        self.state = MsgState()

        self.delta_prev = np.array([0.0, 0.0, 0.0, 0.0])

        self.state = MsgState()

    #creates the update function
    #Arguments:
    #1. f_d: the desired force input to the system
    #2. state: the current state of the system
    def update(self,
               f_d: np.ndarray = np.array([[0.0], [0.0]]),
               state: MsgState = MsgState()):

        self.state = state

        return 0

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
from controllers.momentsCalculator import momentsCalculator



#creates the low level control class
class LowLevelControl:


    def __init__(self):

        #instantiates the moments calculator
        self.momentsCalculator = momentsCalculator()

        pass

    #Arguments:
    #1. the current state
    #2. the desired force on the aircraft in the body frame
    #3. the desired moment exerted on the aircraft in the body frame
    def update(self,
               state: MsgState,
               F_des_b: np.ndarray,
               M_des_b: float):
        

        #gets the elvator delta from the state and the desired M




        pass


    #gets the delta_e saturated, which allows for the maximum possible moment control
    #which may end up being enough to generate a particular moment desired
    def getElevatorThrusters(self,
                         state: MsgState,
                         M_des_b: float):
        
        Va = state.Va
        alpha = state.alpha
        #gets the q nondim
        if Va > 1:
            q_nondim = state.q * CONDA.c / (2 * Va)
        else:
            q_nondim = 0.0

        #gets the four parts of the Moments
        M_0, M_alpha, M_q, M_delta_e = \
              self.momentsCalculator.getAerodynamicMomentsCoefficients(currentState=state)
        
        #from these four parts, get the delta_e unsaturated
        delta_e_unsat = (M_des_b - M_0 - M_alpha*alpha - M_q*q_nondim) / (M_delta_e)

        #saturates so that it stays within 30 degrees (but in radians of course so like +-0.5ish radians)
        delta_e = saturate_delta_e(delta_e_unsat=delta_e_unsat,
                                             bound=CONDA.elevator_bound_rad)

        #with the delta_e, we get the Moment achieved without the thrusters
        M_noThrusters = M_0 + M_alpha*alpha + M_q*q_nondim + M_delta_e*delta_e

        #with these calculations 

        
        #returns the saturated delta_e
        return delta_e





#saturates the delta_e
def saturate_delta_e(delta_e_unsat: float,
                     bound: float = CONDA.elevator_bound_rad)->float:
    
    #min bound is minus bound
    #max bound is bound
    minBound = -bound
    maxBound = bound

    #case delta_e is lower than the min Bound
    if delta_e_unsat < minBound:

        #sets it as the min bound
        delta_e_sat = minBound

    #cae delta_e is greater than the max bound
    elif delta_e_unsat > maxBound:

        #sets it as the max bound
        delta_e_sat = maxBound
    
    #else, it stays itself
    else:

        delta_e_sat = delta_e_unsat

    #returns the saturated delta_e
    return delta_e_sat




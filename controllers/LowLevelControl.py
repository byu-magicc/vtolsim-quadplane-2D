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
        

        #gets the elvator delta and Moment needed from the Thrusters
        delta_e, M_thrusters_des = self.getElevatorThrusters(state=state,
                                                             M_des_b=M_des_b)

        #gets the Force desired in the x and z directions
        F_des_b_x = F_des_b.item(0)
        F_des_b_z = F_des_b.item(1)

        #the Thrust desired for the forward throttle is equal to the F_des_b in the x direction
        T_t_des = F_des_b_x

        #creates the desired forces and moments vector 
        forcesMoments_des = np.array([[F_des_b_z],[M_thrusters_des]])

        #gets the Thrust desired from the vertical 
        verticalThrusts_des = CONDA.individualThrustUnmixer @ forcesMoments_des

        #gets the front and rear vertical thrusts desired
        T_f_des = verticalThrusts_des.item(0)
        T_r_des = verticalThrusts_des.item(1)

        #gets the front Thruster delta
        delta_t_front = inverse_motor_thrust_simplified(Thrust_des=T_f_des)
        #the rear thruster delta
        delta_t_rear = inverse_motor_thrust_simplified(Thrust_des=T_r_des)
        #the forward thruster delta
        delta_t_forward = inverse_motor_thrust_simplified(Thrust_des=T_t_des)

        #creates the delta message class
        deltaOutput = MsgDelta(elevator=delta_e,
                               throttle_front=delta_t_front,
                               throttle_rear=delta_t_rear,
                               throttle_thrust=delta_t_forward)

        return deltaOutput


    #gets the delta_e saturated, which allows for the maximum possible moment control
    #which may end up being enough to generate a particular moment desired
    #Returns:
    #1. delta_e: the elevator saturated control in radians of deflection
    #2. M_thrusters: the Moment remainder needed to be gotten from the thrusters
    def getElevatorThrusters(self,
                         state: MsgState,
                         M_des_b: float)->tuple[float, float]:
        
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

        #with these calculations, we obtain the difference between the M_desired and the M without thrusters
        #that deficit will need to be provided by the difference in thrusters
        M_thrusters_des = M_des_b - M_noThrusters

        
        #returns the saturated delta_e
        return delta_e, M_thrusters_des





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


#defines the inverse motor thrust torque function
#input the desired thrust, get the necessary delta_t
def inverse_motor_thrust_simplified(Thrust_des: float):

    delta_t = Thrust_des / CONDA.MaxThrust
    return delta_t






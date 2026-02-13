import os, sys
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))

import numpy as np
from controllers.feedforwardControl import feedForwardControl, saturate
from message_types.msg_delta import MsgDelta
from message_types.msg_state import MsgState
import parameters.anaconda_parameters as CONDA
from tools.old.rotations import *

import scipy.optimize as spo
from scipy.optimize import minimize
from copy import copy


class LowLevelControl:

    def __init__(self):

        pass

    def update(self,
               state: MsgState,
               F_des_body: np.ndarray,
               M_des_body: float):

        #gets the delta_e and the desired moment from the thrusters
        delta_e, M_thrusters_des = self.getElevatorThrusters(state=state,
                                                             M_des_body=M_des_body)

        #obstains the forces desired in north and down directions
        F_des_n_body = F_des_body.item(0)
        F_des_d_body = F_des_body.item(1)

        #the thrust desired is the thrust forward desired (because they're in the same exact direction)
        T_t_des = F_des_n_body

        #creates the desired forces and moments vector
        forcesMoments_des = np.array([[F_des_d_body],[M_des_body]])

        #gets the vertical thrusts desired
        verticalThrusts_des = CONDA.individualThrustUnmixer @ forcesMoments_des

        #gets the front and rear desired Thrusts
        T_front_des = verticalThrusts_des.item(0)
        T_rear_des = verticalThrusts_des.item(1)

        #gets the front Thruster delta
        delta_t_front = inverse_motor_thrust_simplified(Thrust_des=T_front_des)
        #the rear thruster delta
        delta_t_rear = inverse_motor_thrust_simplified(Thrust_des=T_rear_des)
        #the forward thruster delta
        delta_t_forward = inverse_motor_thrust_simplified(Thrust_des=T_t_des)

        deltaOutput = MsgDelta(elevator=delta_e,
                               throttle_front=delta_t_front,
                               throttle_rear=delta_t_rear,
                               throttle_thrust=delta_t_forward)

        #returns the delta output
        return deltaOutput

    #gets the delta_e saturated, which allows for the maximum possible moment control
    #which may end up being enough to generate a particular moment desired
    #Returns:
    #1. delta_e: the elevator saturated control in radians of deflection
    #2. M_thrusters: the Moment remainder needed to be gotten from the thrusters
    def getElevatorThrusters(self,
                             state: MsgState,
                             M_des_body: float):

        Va = state.Va
        alpha = state.alpha

        #gets the q nondim
        if Va > 1:
            q_nondim = state.q * CONDA.c / (2 * Va)
        else:
            q_nondim = 0.0

        #gets the four parts of the moments (but not the actual moment itself)
        M_0, M_q, M_delta_e = self.getAerodynamicMomentsCoefficients(currentState=state)

        #from these four parts, we get the delta_e desired, but unsaturated
        #(potentially past the allowable limits of what delta_e can handle)
        delta_e_unsat = (M_des_body - M_0 - M_q*q_nondim)/(M_delta_e)

        #gets delta_e saturated
        delta_e = saturate_delta_e(delta_e_unsat=delta_e_unsat,
                                   bound=CONDA.elevator_bound_rad)

        #with the delta_e, we get the Moment achieved without the thrusters
        M_noThrusters = M_0 + M_q*q_nondim + M_delta_e*delta_e

        #with these calculations, we obtain the difference between the M_desired and the M without thrusters
        #that deficit will need to be provided by the difference in thrusters
        M_thrusters_des = M_des_body - M_noThrusters
        
        #returns the saturated delta_e
        return delta_e, M_thrusters_des


    #because the aerodynamic Moment is broken up into several
    #smaller terms that are summed together, and in this case,
    #we want to get each term. 
    #IN OTHER WORDS: The aircraft moment can be broken into 
    #three components:
    #1. uncontrollable aerodynamic moment (purely based on state from airspeed and alpha and q)
    #2. controllable aerodynamic moment (controlled by elevator input)
    #3. rotor aerodynamic moment (caused by differential control of two vertical rotors.)
    #for this the notation is a bit sticky.
    #M_0 returns q_bar_total*(Cm0 + alpha*Cmalpha)
    #M_q returns q_bar_total*(Cmq*c/(2Va))
    def getAerodynamicMomentsCoefficients(self,
                              currentState: MsgState):
        
        #gets the airspeed from the current state
        Va = currentState.Va

        alpha = currentState.alpha

        #gets the q_bar 
        qbar = 0.5 * CONDA.rho * Va**2

        #gets the qbar total
        qbar_total = qbar * CONDA.S_wing * CONDA.c

        #from this gets the individual aerodynamic moments

        #the natural moment 
        M_0 = qbar_total * (CONDA.C_m_0 + CONDA.C_m_alpha*alpha) 

        #the q Moment
        M_q = qbar_total*CONDA.C_m_q

        #the delta moment
        M_delta_e = qbar_total * CONDA.C_m_delta_e

        #returns all four of these
        return M_0, M_q, M_delta_e


def saturate_delta_e(delta_e_unsat: float,
                     bound: float = CONDA.elevator_bound_rad):

    minBound = -bound
    maxBound = bound

    #inserts the cases for possible saturations on delta_e
    if delta_e_unsat < minBound:
        delta_e_sat = minBound
    elif delta_e_unsat > maxBound:
        delta_e_sat = maxBound
    else:
        delta_e_sat = delta_e_unsat

    return delta_e_sat


#defines the inverse motor thrust torque function
#input the desired thrust, get the necessary delta_t
def inverse_motor_thrust_simplified(Thrust_des: float):

    delta_t = Thrust_des / CONDA.MaxThrust
    return delta_t

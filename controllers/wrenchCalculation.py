#This file implements the wrench calculator for the low level controller
import numpy as np
from message_types.msg_delta import MsgDelta
from message_types.msg_state import MsgState

from tools.rotations import rotation_to_theta_2d, theta_to_rotation_2d, alphaToRotation

import parameters.anaconda_parameters as QP

#This iteration will NOT take into account gravity. This will only take
#into account the aerodynamic and actuator forces on the aircraft, in 2 Dimensions
class wrenchCalculator:

    #creates the initialization function
    def __init__(self):

        #creates a counter
        self.counter = 0

    

    #creates the forces moments and the Jacobian for it
    def forces_moments_derivatives(self, delta: MsgDelta, state: MsgState):

        temp = 0

        #gets th


    #creates the function to get the actual force and torque acheived, WITHOUT gravity
    def forces_moments_achieved(self, delta: MsgDelta, state: MsgState):
        #gets the theta from the state 
        theta = rotation_to_theta_2d(R=state.R)

        #gets the rotation matrix from the body to the inertial
        R_body2Inertial = theta_to_rotation_2d(theta=theta)
        #gets the rotation matrix from the inertial to the body
        R_inertial2Body = np.transpose(R_body2Inertial)

        # pitch rate
        q = state.omega.item(0)

        #gets the alpha
        alpha = state.alpha

        #gets the Va
        Va = state.Va

        #gets the v_air
        v_air = state.v_air

        #intermediate variables
        qbar = 0.5 * QP.rho * Va**2
        #gets the cosine of alpha
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        if Va > 1:
            q_nondim = q * QP.c / (2 * Va)
        else:
            q_nondim = 0.0


        # compute Lift and Drag coefficients
        tmp1 = np.exp(-QP.M * (alpha - QP.alpha0))
        tmp2 = np.exp(QP.M * (alpha + QP.alpha0))
        sigma = (1 + tmp1 + tmp2) / ((1 + tmp1) * (1 + tmp2))
        CL = (1 - sigma) * (QP.C_L_0 + QP.C_L_alpha * alpha) \
             + sigma * 2 * np.sign(alpha) * sa**2 * ca
        CD = QP.C_D_p + ((QP.C_L_0 + QP.C_L_alpha * alpha)**2)/(np.pi * QP.e * QP.AR)

        # compute Lift and Drag Forces
        F_lift = qbar * QP.S_wing * (CL + QP.C_L_q * q_nondim + QP.C_L_delta_e * delta.elevator)
        F_drag = qbar * QP.S_wing * (CD + QP.C_D_q * q_nondim + QP.C_D_delta_e * delta.elevator)

        #gets the forces in the body frame
        f_body = alphaToRotation(alpha=alpha) @ np.array([[F_lift],
                                                          [F_drag]])
        
        # compute pitching moment 
        My = qbar * QP.S_wing * QP.c * (
                QP.C_m_0
                + QP.C_m_alpha * alpha
                + QP.C_m_q * q_nondim
                + QP.C_m_delta_e * delta.elevator
        )

        #the front vertically oriented prop
        Va_front_prop = self.v_air_body.item(1)
        #the rear vertically oriented prop
        Va_rear_prop = self.v_air_body.item(1)
        #the forward oriented prop, generating the main thrust
        Va_forward_prop = -self.v_air_body.item(0)


        # compute forces and torques from each propeller
        Thrust_front, Q_f = self._motor_thrust_torque(Va_front_prop, delta.throttle_front)
        Thrust_rear, Q_r = self._motor_thrust_torque(Va_rear_prop, delta.throttle_rear)
        Thrust_forward, Q_t = self._motor_thrust_torque(Va_forward_prop, delta.throttle_thrust)

        # add propeller forces and torques to body
        f_body[0][0] += Thrust_forward
        f_body[1][0] += -Thrust_front - Thrust_rear
        My += QP.ell_f * Thrust_front - QP.ell_r * Thrust_rear

        #returns the body frame forces and moments
        return np.array([[f_body.item(0)],
                         [f_body.item(1)],
                         [My]])
    


    #gets the wrench Jacobian
    def wrench_Jacobian(self, delta: MsgDelta, state: MsgState):

        



    def _motor_thrust_torque(self, Va: float, delta_t: float)->tuple[float, float]:
        C_Q0 = QP.C_Q0
        C_Q1 = QP.C_Q1
        C_T0 = QP.C_T0
        C_Q2 = QP.C_Q2
        C_T1 = QP.C_T1
        C_T2 = QP.C_T2
        D_prop = QP.D_prop
        KQ = QP.KQ
        R_motor = QP.R_motor
        i0 = QP.i0
        #gets the voltage in, based on the delta_t
        V_in = QP.V_max * delta_t
        # Quadratic formula to solve for motor speed
        a = C_Q0 * QP.rho * np.power(D_prop, 5)/((2.*np.pi)**2)
        b = (C_Q1 * QP.rho * np.power(D_prop, 4)/ (2.*np.pi)) * Va + KQ**2/R_motor
        c = C_Q2 * QP.rho * np.power(D_prop, 3)* Va**2 - (KQ / R_motor) * V_in + KQ * i0        
        # Consider only positive root
        Omega_op = (-b + np.sqrt(b**2 - 4*a*c)) / (2.*a)
        # compute advance ratio
        J_op = 2 * np.pi * Va / (Omega_op * D_prop)
        # compute non-dimensionalized coefficients of thrust and torque
        C_T = C_T2 * J_op**2 + C_T1 * J_op + C_T0
        C_Q = C_Q2 * J_op**2 + C_Q1 * J_op + C_Q0
        # add thrust and torque due to propeller
        n = Omega_op / (2 * np.pi)
        T_p = QP.rho * n**2 * np.power(D_prop, 4) * C_T
        Q_p = QP.rho * n**2 * np.power(D_prop, 5) * C_Q
        return T_p, Q_p   
    

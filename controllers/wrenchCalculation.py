#This file implements the wrench calculator for the low level controller
import numpy as np
from message_types.msg_delta import MsgDelta
from message_types.msg_state import MsgState

from tools.rotations import *

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

        #gets the forces and moments achieved
        wrenchAchieved = self.forces_moments_achieved(delta=delta, state=state)

        #gets the jacobian
        wrenchJacobian = self.wrench_Jacobian(delta=delta, state=state)

        #returns the wrench achieved and wrench Jacobain 
        return wrenchAchieved, wrenchJacobian




    #creates the function to get the actual force and torque acheived, WITHOUT gravity
    def forces_moments_achieved(self, delta: MsgDelta, state: MsgState):
        #gets the theta from the state 
        theta = state.theta

        #gets the rotation matrix from the body to the inertial
        R_body2Inertial = theta_to_rotation_2d(theta=theta)
        #gets the rotation matrix from the inertial to the body
        R_inertial2Body = np.transpose(R_body2Inertial)

        # pitch rate
        q = state.q

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
        f_body = alphaToRotation(alpha=alpha) @ np.array([[F_drag],
                                                          [F_lift]])
        
        # compute pitching moment 
        My = qbar * QP.S_wing * QP.c * (
                QP.C_m_0
                + QP.C_m_alpha * alpha
                + QP.C_m_q * q_nondim
                + QP.C_m_delta_e * delta.elevator
        )

        #the front vertically oriented prop
        Va_front_prop = -state.v_air.item(1)
        #the rear vertically oriented prop
        Va_rear_prop = -state.v_air.item(1)
        #the forward oriented prop, generating the main thrust
        Va_forward_prop = state.v_air.item(0)


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
        #gets the gamma variable
        Gamma = (1/2)*QP.rho*((state.Va)**2)*QP.S_wing
        #gets the alpha variable
        alpha = state.alpha

        #gets the alpha rotation matrix
        alpha_rotation = alphaToRotation(alpha=alpha)

        #creates the vector of the partials of the Lift and Drag Forces
        partial_F_drag = Gamma*QP.C_D_delta_e
        partial_F_lift = Gamma*QP.C_L_delta_e

        liftDragPartials = np.array([[partial_F_drag],
                                     [partial_F_lift]])
        
        #gets the rotated partials
        rotatedPartials = alpha_rotation @ liftDragPartials
        #gets the two components of the above vector and breaks them up
        partial_fx_delta_e = rotatedPartials.item(0)
        partial_fz_delta_e = rotatedPartials.item(1)
        #gets the partial of My with respect to delta_e
        partial_My_delta_e = Gamma*QP.c*QP.C_m_delta_e


        #gets the rotor forces, moments, and their derivitives lists
        rotorForces, rotorMoments, rotorForceDerivatives, rotorMomentDerivatives = \
                            self.rotor_thrust_torque_derivatives(delta=delta, state=state)

        
        #gets the partial for the forces
        partial_T_delta_t1 = rotorForceDerivatives[0]
        partial_T_delta_t2 = rotorForceDerivatives[1]
        partial_T_delta_t3 = rotorForceDerivatives[2]

        #creates the gradient vector for each wrench component
        fx_gradient = [partial_fx_delta_e, 
                       0,
                       0,
                       partial_T_delta_t3]
        
        #creates the z gradient
        fz_gradient = [partial_fz_delta_e,
                       -partial_T_delta_t1,
                       -partial_T_delta_t2,
                       0]
        
        My_gradient = [partial_My_delta_e,
                       QP.ell_f*partial_T_delta_t1,
                       -QP.ell_r*partial_T_delta_t2,
                       0]
        

        #returns a vector of the gradients
        return np.array([fx_gradient, fz_gradient, My_gradient]).T



    #function to calculate the motor thrust and aerodynamic torque or moment
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
    



    #creates the function to get the rotor thrust torques and derivatives of thrust and torque.
    #this function is structured so that it obtains the information for all three rotors, and 
    #returns the information as a vector

    def rotor_thrust_torque_derivatives(self, delta: MsgDelta, state: MsgState)->tuple[list, list, list, list]:
        
        #gets the airspeed vector
        v_air = state.v_air

        numProps = 3
    

        #gets the airspeed through the propellers
        airmass_velocity_rotors = np.array([-v_air.item(1),
                                            -v_air.item(1),
                                            v_air.item(0)])


        #creates the array for the three throttles
        delta_throttles = np.array([delta.throttle_front,
                                    delta.throttle_rear,
                                    delta.throttle_thrust])
        
        #creates the list for the thrusts, torques, thrust_derivatives, and torque derivatives
        thrusts = []
        torques = []
        thrust_derivatives = []
        torque_derivatives = []
        
        #iterates through the three props
        for i in range(numProps):
            #saves the coefficients of aerodynamics for the propeller model
            C_Q0 = QP.C_Q0
            C_Q1 = QP.C_Q1
            C_T0 = QP.C_T0
            C_Q2 = QP.C_Q2
            C_T1 = QP.C_T1
            C_T2 = QP.C_T2

            #sets the diameter of the propeller
            D_prop = QP.D_prop
            KQ = QP.KQ
            R_motor = QP.R_motor
            i0 = QP.i0

            # map delta_t throttle command(0 to 1) into motor input voltage
            V_in = QP.V_max * delta_throttles[i]
            V_in_der = QP.V_max

            # Quadratic formula to solve for motor speed
            a = C_Q0 * QP.rho * np.power(D_prop, 5) \
                / ((2.*np.pi)**2)
            b = (C_Q1 * QP.rho * np.power(D_prop, 4)
                / (2.*np.pi)) * airmass_velocity_rotors[i] + KQ**2/R_motor
            c = C_Q2 * QP.rho * np.power(D_prop, 3) \
                * (airmass_velocity_rotors[i])**2 - (KQ / R_motor) * V_in + KQ * i0
            c_der = (KQ / R_motor) * V_in_der

            # Consider only positive root
            Omega_op = (-b + np.sqrt(b**2 - 4*a*c)) / (2.*a)
            Omega_op_der = c_der / np.sqrt(b**2 - 4*a*c)

            # compute advance ratio
            J_op = 2 * np.pi * airmass_velocity_rotors[i] / (Omega_op * D_prop)
            J_op_der = -2 * np.pi * airmass_velocity_rotors[i] * Omega_op_der / (Omega_op**2 * D_prop)

            # compute non-dimensionalized coefficients of thrust and torque
            C_T = C_T2 * J_op**2 + C_T1 * J_op + C_T0
            C_Q = C_Q2 * J_op**2 + C_Q1 * J_op + C_Q0
            C_T_der = 2 * C_T2 * J_op * J_op_der + C_T1 * J_op_der
            C_Q_der = 2 * C_Q2 * J_op * J_op_der + C_Q1 * J_op_der

            # add thrust and torque due to propeller
            n = Omega_op / (2 * np.pi)
            T_p = QP.rho * n**2 * np.power(D_prop, 4) * C_T
            Q_p = QP.rho * n**2 * np.power(D_prop, 5) * C_Q
            T_p_der = QP.rho * Omega_op * Omega_op_der * np.power(D_prop, 4) * C_T / (2 * np.pi**2) + \
                QP.rho * Omega_op**2 * np.power(D_prop, 4) * C_T_der / (2 * np.pi)**2
            Q_p_der = QP.rho * Omega_op * Omega_op_der * np.power(D_prop, 5) * C_Q / (2 * np.pi**2) + \
                QP.rho * Omega_op**2 * np.power(D_prop, 5) * C_Q_der / (2 * np.pi)**2
            
            #changes the direction
            Q_p = (QP.propDirections)[i]*Q_p

            #appends to the lists
            thrusts.append(T_p)
            torques.append(Q_p)
            thrust_derivatives.append(T_p_der)
            torque_derivatives.append(Q_p_der)

        #returns the arrays
        return thrusts, torques, thrust_derivatives, torque_derivatives
            
    
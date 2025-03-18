#quadplane dynamics - With the traditional thrust calculation, not the simplified one.
#-this file implements the dynamic equations of motion for the quad plane
#uses unit quaternion for the attitude state
import numpy as np
import parameters.anaconda_parameters as QP
from tools.rotations import *
from tools.quaternions import *
from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta


#creates the QuadplaneDynamics class
class QuadplaneDynamics:
    #creates the initialization function
    #saves the timestep for simulation, as well as the 
    #bool to enable whether the quadrotors will be enabled or disabled in the dynamics
    def __init__(self, 
                 ts: float,
                 pn0: float=QP.pn0,
                 pd0: float=QP.pd0,
                 pn_dot0: float=QP.pn_dot0,
                 pd_dot0: float=QP.pd_dot0,
                 theta0: float=QP.theta0,
                 q0: float=QP.q0,
                 ):
        self._ts = ts
        #creates the state array and initializes them to the original positions
        self._state = np.array([
            [pn0],    # [0]  north position inertial
            [pd0],    # [1]  down position inertial
            [pn_dot0],     # [2]  velocity along body x-axis
            [pd_dot0],     # [3]  velocity along body z-axis
            [theta0], # [4] initial pitch angle
            [q0],     # [5]  pitch rate
        ])
        self.true_state = MsgState()
        self._update_velocity_data()
        self._forces_moments(delta=MsgDelta())
        # update the message class for the true state
        self._update_true_state()

        #creates the v_air vector, which is an airspeed vector, in the body frame.
        #it is the velocity of the aircraft with respect to the airmass
        #in which it is travelling through. 
        self.v_air_inertial = np.array([[QP.pn_dot0],
                                        [QP.pd_dot0]])
        self.v_air_body = np.array([[QP.pn_dot0],
                                    [QP.pd_dot0]])
        
        potato = 0

    #creates the update function for the system
    def update(self, delta: MsgDelta, wind: np.ndarray):
        #calls to get the forces and the moments of the system
        forces_moments = self._forces_moments(delta)
        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self._ts
        k1 = self._f(self._state, forces_moments)
        k2 = self._f(self._state + time_step/2.*k1, forces_moments)
        k3 = self._f(self._state + time_step/2.*k2, forces_moments)
        k4 = self._f(self._state + time_step*k3, forces_moments)
        self._state += time_step/6 * (k1 + 2*k2 + 2*k3 + k4)
        # update the airspeed and angle of attack using new state
        self._update_velocity_data(wind)
        # update the message class for the true state
        self._update_true_state()

    ###################################
    # private functions
    def _f(self,
            state: np.ndarray, 
            forces_moments: np.ndarray):

        # extract the states
        # pn = state.item(0)
        # pd = state.item(1)

        #u and w are the body frame velocities
        pn = state.item(0)
        pd = state.item(1)
        pn_dot = state.item(2)
        pd_dot = state.item(3)
        theta = state.item(4)
        q = state.item(5)

        #   extract forces/moments in body frame
        fx_body = forces_moments.item(0)
        fz_body = forces_moments.item(1)
        My = forces_moments.item(2)
        # position kinematics

        #gets the rotation matrix from body to inertial
        R_body2inertial = theta_to_rotation_2d(theta=theta)
        #from inertial to body
        R_inertial2body = np.transpose(R_body2inertial)

        #gets the forces in the inertial frame
        f_inertial = R_body2inertial @ np.array([[fx_body],
                                                 [fz_body]])

        #
        # position dynamics
        pn_ddot = (1/QP.mass)*f_inertial.item(0)
        pd_ddot = (1/QP.mass)*f_inertial.item(1)
        #print('F=', f_inertial.item(0), ', ', f_inertial.item(1))
        #print('M=', My)
        # rotational kinematics
        theta_dot = q
        # rotatonal dynamics
        q_dot = My/QP.Jy
        # collect the derivative of the states
        x_dot = np.array([[pn_dot, pd_dot, pn_ddot, pd_ddot, theta_dot, q_dot]]).T
        return x_dot

    #creates the update velocity data function
    def _update_velocity_data(self, wind: np.ndarray=np.zeros((4,1))):
        #gets the steady state wind
        steadyStateWind_inertial = wind[0:2]
        gust = wind[2:4]
        # convert wind vector from world to body frame
        theta = self._state.item(4)
        #gets the rotation from body to inertial
        R_body2inertial = theta_to_rotation_2d(theta=theta)
        #gets the Rotation from inertial to body
        R_inertial2body = R_body2inertial.T

        #
        wind_body = R_inertial2body @ steadyStateWind_inertial # rotate steady state wind to body frame
        wind_body += gust  # add the gust
        wind_inertial = R_body2inertial @ wind_body  # wind in the world frame
        self._wind = wind_inertial



        #gets the velocity in the inertial frame
        velocity_inertial_frame = self._state[2:4]
        #gets the air velocity with respect to the inertial frame
        self.v_air_inertial = velocity_inertial_frame - wind_inertial

        #gets the v air in the body frame
        self.v_air_body = R_inertial2body @ self.v_air_inertial
        #computes the actual airspeed
        # compute airspeed
        self._Va = np.linalg.norm( self.v_air_body )
        ur = self.v_air_body.item(0)
        wr = self.v_air_body.item(1)
        # compute angle of attack
        if ur==0:
            self._alpha = np.sign(wr)*np.pi/2.
        else:
            self._alpha = np.arctan(wr/ur)
        #computes the groundspeed
        self._Vg = np.linalg.norm(velocity_inertial_frame)
        
    #creates the forces moments function
    def _forces_moments(self, delta: MsgDelta)->np.ndarray:
        # rotation from body to world frame
        theta = self._state.item(4)

        #gets the rotation matrix from the body to the inertial
        R_body2Inertial = theta_to_rotation_2d(theta=theta)
        #gets the rotation matrix from the inertial to the body
        R_inertial2Body = np.transpose(R_body2Inertial)


        # pitch rate
        q = self._state.item(5)
        #gets the force of gravity in the inertial frame
        fg_inertial = QP.mass * QP.gravity_accel_inertial
        # gravitational force in body frame
        f_g_body = R_inertial2Body @ fg_inertial
        # gets each portion of the gravitational force, and creates the fx and fz body components
        fx_body = f_g_body.item(0)
        fz_body = f_g_body.item(1)
        #intermediate variables
        qbar = 0.5 * QP.rho * self._Va**2
        ca = np.cos(self._alpha)
        sa = np.sin(self._alpha)
        # nondimensionalize q
        if self._Va > 1:
            q_nondim = q * QP.c / (2 * self._Va)
        else:
            q_nondim = 0.0
        # compute Lift and Drag coefficients
        tmp1 = np.exp(-QP.M * (self._alpha - QP.alpha0))
        tmp2 = np.exp(QP.M * (self._alpha + QP.alpha0))
        sigma = (1.0 + tmp1 + tmp2) / ((1.0 + tmp1) * (1.0 + tmp2))
        CL = (1.0 - sigma) * (QP.C_L_0 + QP.C_L_alpha * self._alpha) \
             + sigma * 2 * np.sign(self._alpha) * sa**2 * ca
        CD = QP.C_D_p + ((QP.C_L_0 + QP.C_L_alpha * self._alpha)**2)/(np.pi * QP.e * QP.AR)
        # compute Lift and Drag Forces
        F_lift = qbar * QP.S_wing * (CL + QP.C_L_q * q_nondim + QP.C_L_delta_e * delta.elevator)
        F_drag = qbar * QP.S_wing * (CD + QP.C_D_q * q_nondim + QP.C_D_delta_e * delta.elevator)
        # compute longitudinal forces in body frame
        fx_body += - ca * F_drag + sa * F_lift
        fz_body += - sa * F_drag - ca * F_lift
        
        fx_body_aerodynamic = - ca * F_drag + sa * F_lift
        fz_body_aerodynamic = - sa * F_drag - ca * F_lift
        #adds it
        fx_body += fx_body_aerodynamic
        fz_body += fz_body_aerodynamic
        # compute pitching moment 
        My = qbar * QP.S_wing * QP.c * (
                QP.C_m_0
                + QP.C_m_alpha * self._alpha
                + QP.C_m_q * q_nondim
                + QP.C_m_delta_e * delta.elevator)
        
        #gets the airspeed through the 
        # compute airspeed through each propeller
        #remember that it is the velocity of the air coming at the front of the prop,
        #which is the negative of the airspeed of the aircraft going through the air

        #the front vertically oriented prop
        Va_front_prop = -self.v_air_body.item(1)
        #the rear vertically oriented prop
        Va_rear_prop = -self.v_air_body.item(1)
        #the forward oriented prop, generating the main thrust
        Va_forward_prop = self.v_air_body.item(0)

        # compute forces and torques from each propeller
        Thrust_front, Q_0 = self._motor_thrust_torque_advanced(delta_t=delta.throttle_front,
                                                               Va=Va_front_prop)
        Thrust_rear, Q_1 = self._motor_thrust_torque_advanced(delta_t=delta.throttle_rear,
                                                              Va=Va_rear_prop)
        Thrust_forward, Q_2 = self._motor_thrust_torque_advanced(delta_t=delta.throttle_thrust,
                                                                 Va=Va_forward_prop)
        # add propeller forces and torques to body
        fx_body += Thrust_forward
        fz_body += -Thrust_front - Thrust_rear
        My += QP.ell_f * Thrust_front - QP.ell_r * Thrust_rear
        #returns the forces
        return np.array([[fx_body, fz_body, My]]).T
    

    #creates a wrapper function for the forces and moments
    #has a delta array instead of a message
    def forces_moments_wrapper(self, deltaArray: np.ndarray):
        #creates the delta message
        deltaMessage = MsgDelta()
        deltaMessage.from_array(delta_array=deltaArray)
        #runs the _forces_moments function
        forcesMoments = (self._forces_moments(delta=deltaMessage))[:,0]

        return forcesMoments


    #creates the new motor thrust and torque function, which is an over simplification,
    #but is better for our simpler modelling and figuring everything out
    def _motor_thrust_torque_simplified(self, delta_t: float):
        #returns the delta_t times the max thrust
        return delta_t*QP.MaxThrust


    def _motor_thrust_torque_advanced(self, Va: float, delta_t: float)->tuple[float, float]:
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

    #function to update the true state
    def _update_true_state(self):

        #gets the components of the vector state
        #gets the positions
        pn = self._state.item(0)
        pd = self._state.item(1)
        #gets the body frame velocities
        pn_dot = self._state.item(2)
        pd_dot = self._state.item(3)
        #gets the pitch
        theta = self._state.item(4)
        #gets the pitch rate
        q = self._state.item(5)

        #saves them all to the respective variables
        self.true_state.pos = np.array([[pn],
                                        [0.0],
                                        [pd]])
        self.true_state.vel = np.array([[pn_dot],
                                        [0.0], 
                                        [pd_dot]])
        self.true_state.R = theta_to_rotation_3d(theta=theta)
        self.true_state.omega = np.array([[0.0],
                                          [q],
                                          [0.0]])
        #updates the airspeed magnitude and the angle of attack
        self.true_state.Va = self._Va
        self.true_state.alpha = self._alpha
        #updates the airspeed vector
        self.true_state.v_air = self.v_air_body
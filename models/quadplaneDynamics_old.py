#the quadplane dynamics file

import numpy as np
import parameters.anaconda_parameters as CONDA
import parameters.simulation_parameters as SIM
from tools.rotations import *

from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta

from rrt_mavsim.message_types.msg_plane import MsgPlane
from rrt_mavsim.tools.plane_projections import *



#creates the Quadplane dynamics class
class QuadplaneDynamics:

    #creates the initialization function
    def __init__(self,
                 plane_msg: MsgPlane,
                 ts: float = SIM.ts_simulation,
                 pos_3D_0: np.ndarray = np.array([[0.0],[0.0],[0.0]]),
                 vel_3D_0: np.ndarray = np.array([[0.0],[0.0],[0.0]]),
                 theta0: float=CONDA.theta0,
                 q0: float=CONDA.q0):
        
        self.counter = 0

        self.counterInterval = 10

        self.plane_msg = plane_msg
        self.ts = ts

        #gets the 3D position and Projects it onto the respective plane
        pos_2D_init = map_3D_to_2D_planeMsg(vec_3D=pos_3D_0,
                                            plane_msg=plane_msg)
        
        vel_2D_init = map_3D_to_2D_planeMsg(vec_3D=vel_3D_0,
                                            plane_msg=plane_msg)


        #creates the state array from the initialized 2D points
        self._state = np.concatenate((pos_2D_init, vel_2D_init, np.array([[theta0]]), np.array([[q0]])), axis=0)
        

        #creates the true state
        self.true_state = MsgState(plane_msg=self.plane_msg)
        #updates the vleocity data, as in the wind and everything else 
        # (assume no wind for now)
        self._update_velocity_data()
        #initially calls the forces momemnts function
        self._forces_moments(delta=MsgDelta())
        #updates the true state
        self._update_true_state()

        potato = 0

    #creates the update function for the system
    def update(self, 
               delta: MsgDelta, 
               wind: np.ndarray):
        #calls to get the forces and the moments of the system
        forces_moments = self._forces_moments(delta)
        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self.ts
        k1 = self._f(self._state, forces_moments)
        k2 = self._f(self._state + time_step/2.*k1, forces_moments)
        k3 = self._f(self._state + time_step/2.*k2, forces_moments)
        k4 = self._f(self._state + time_step*k3, forces_moments)
        self._state = self._state + time_step/6 * (k1 + 2*k2 + 2*k3 + k4)
        # update the airspeed and angle of attack using new state
        self._update_velocity_data(wind=wind)
        # update the message class for the true state
        self._update_true_state()

    #defines the f function
    def _f(self,
           state: np.ndarray,
           forces_moments: np.ndarray):
        
        pn_dot = state.item(2)
        pu_dot = state.item(3)
        theta = state.item(4)
        q = state.item(5)

        fx_body = forces_moments.item(0)
        fz_body = forces_moments.item(1)
        My = forces_moments.item(2)

        R_body2inertial = theta_to_rotation_2D(theta=theta)

        f_inertial = R_body2inertial @ np.array([[fx_body],[fz_body]])

        pn_ddot = (1/CONDA.mass)*f_inertial.item(0)
        pu_ddot = (1/CONDA.mass)*f_inertial.item(1)

        theta_dot = q

        q_dot = My/CONDA.Jy

        x_dot = np.array([[pn_dot, pu_dot, pn_ddot, pu_ddot, theta_dot, q_dot]]).T

        return x_dot

    #creates the forces and moments
    def _forces_moments(self, delta: MsgDelta)->np.ndarray:

        #gets the rotation from body to the world frame
        theta = self._state.item(4)
        #gets the rotation matrix from the body to the inertial
        R_body2Inertial_2D = theta_to_rotation_2D(theta=theta)
        #gets the rotation matrix from the inertial to the body
        R_inertial2Body_2D = np.transpose(R_body2Inertial_2D)

        #gets the pitch rate
        q = self._state.item(5)

        #gets the force of gravity in the inertial frame
        fg_inertial_3D = CONDA.mass * CONDA.gravity * CONDA.e3_3D
        #gets it in the 2D frame
        fg_inertial_2D = map_3D_to_2D_planeMsg(vec_3D=fg_inertial_3D,
                                               plane_msg=self.plane_msg)
        #rotates it into the body frame
        fg_body_2D = R_inertial2Body_2D @ fg_inertial_2D

        #initializes the main forces fx and fz in the body frame.
        #we will add to this as the function progresses. This is the first term.
        fx_body = fg_body_2D.item(0)
        fz_body = fg_body_2D.item(1)

        #q_bar is the coefficient out front of all the aerodynamic forces and moments
        qbar = 0.5 * CONDA.rho * self._Va**2

        #gets the cosine and sine of alpha
        ca = np.cos(self._alpha)
        sa = np.sin(self._alpha)

        # nondimensionalize q
        if self._Va > 1:
            q_nondim = q * CONDA.c / (2 * self._Va)
        else:
            q_nondim = 0.0

        # compute Lift and Drag coefficients
        tmp1 = np.exp(-CONDA.M * (self._alpha - CONDA.alpha0))
        tmp2 = np.exp(CONDA.M * (self._alpha + CONDA.alpha0))
        sigma = (1.0 + tmp1 + tmp2) / ((1.0 + tmp1) * (1.0 + tmp2))
        CL = (1.0 - sigma) * (CONDA.C_L_0 + CONDA.C_L_alpha * self._alpha) \
             + sigma * 2 * np.sign(self._alpha) * sa**2 * ca
        CD = CONDA.C_D_p + ((CONDA.C_L_0 + CONDA.C_L_alpha * self._alpha)**2)/(np.pi * CONDA.e * CONDA.AR)
        # compute Lift and Drag Forces
        F_lift = qbar * CONDA.S_wing * (CL + CONDA.C_L_q * q_nondim + CONDA.C_L_delta_e * delta.elevator)
        F_drag = qbar * CONDA.S_wing * (CD + CONDA.C_D_q * q_nondim + CONDA.C_D_delta_e * delta.elevator)

        #adds the Lift and Drag to the components
        fx_body += -ca * F_drag + sa * F_lift
        fz_body += sa * F_drag + ca * F_lift

        #computes the pitching moment
        My = qbar * CONDA.S_wing * CONDA.c * (
                CONDA.C_m_0
                + CONDA.C_m_alpha * self._alpha
                + CONDA.C_m_q * q_nondim
                + CONDA.C_m_delta_e * delta.elevator)

        Thrust_front = self._motor_thrust_torque_simplified(delta_t=delta.throttle_front)
        Thrust_rear = self._motor_thrust_torque_simplified(delta_t=delta.throttle_rear)
        Thrust_forward = self._motor_thrust_torque_simplified(delta_t=delta.throttle_thrust)

        #adds the thrust forward to the fx body
        fx_body += Thrust_forward
        fz_body += Thrust_front + Thrust_rear
        My += CONDA.ell_f * Thrust_front - CONDA.ell_r * Thrust_rear


        if self.counter % self.counterInterval == 0:

            potato = 0

        self.counter += 1

        return np.array([[fx_body],[fz_body],[My]])

    #the update velocity data function
    def _update_velocity_data(self,
                              wind: np.ndarray=np.zeros((4,1))):
        #gets the steady state wind from the input variable
        #gets the steady state wind
        steadyStateWind_inertial = wind[0:2]
        gust = wind[2:4]

        theta=self._state.item(4)

        #gets the Rotation from Body to inertial
        R_body2inertial = theta_to_rotation_2D(theta=theta)
        #and it's inverse
        R_inertial2body = R_body2inertial.T

        #gets the wind in the body frame
        wind_body = R_inertial2body @ steadyStateWind_inertial
        wind_body += gust
        wind_inertial = R_body2inertial @ wind_body
        self._wind = wind_inertial


        #gets the velocity in the inertial frame
        velocity_inertial_frame = self._state[2:4,0:1]
        #gets the air velocity with respect to the inertial frame
        self.v_air_inertial = velocity_inertial_frame - wind_inertial
        #gets the v air in the body frame
        self.v_air_body = R_inertial2body @ self.v_air_inertial

        #gets the airspeed magnitude
        self._Va = np.linalg.norm(self.v_air_body)
        ur = self.v_air_body.item(0)
        wr = self.v_air_body.item(1)
        # compute angle of attack
        if ur==0:
            self._alpha = -np.sign(wr)*np.pi/2.
        else:
            self._alpha = np.arctan2(-wr, ur)

        potato = 0


    def _motor_thrust_torque(self, Va: float, delta_t: float) -> tuple[float, float]:
        '''
        compute thrust and torque due to propeller
        '''
        # map delta_t throttle command(0 to 1) into motor input voltage
        v_in = CONDA.V_max * delta_t
        # Quadratic formula to solve for motor speed
        a = CONDA.C_Q0 * CONDA.rho * np.power(CONDA.D_prop, 5) \
            / ((2.*np.pi)**2)
        b = (CONDA.C_Q1 * CONDA.rho * np.power(CONDA.D_prop, 4)
             / (2.*np.pi)) * Va + CONDA.KQ * CONDA.KV / CONDA.R_motor
        c = CONDA.C_Q2 * CONDA.rho * np.power(CONDA.D_prop, 3) \
            * Va**2 - (CONDA.KQ / CONDA.R_motor) * v_in + CONDA.KQ * CONDA.i0
       
        # Angular speed of propeller
        omega_p = (-b + np.sqrt(b**2 - 4*a*c)) / (2.*a)
        # compute advance ratio
        J_p = 2 * np.pi * Va / (omega_p * CONDA.D_prop)
        # compute non-dimensionalized coefficients of thrust and torque
        C_T = CONDA.C_T2 * J_p**2 + CONDA.C_T1 * J_p + CONDA.C_T0
        C_Q = CONDA.C_Q2 * J_p**2 + CONDA.C_Q1 * J_p + CONDA.C_Q0
        # compute propeller thrust and torque
        n = omega_p / (2 * np.pi)
        thrust_prop = CONDA.rho * n**2 * np.power(CONDA.D_prop, 4) * C_T
        torque_prop = CONDA.rho * n**2 * np.power(CONDA.D_prop, 5) * C_Q

        #the power being consumed by the propeller is omega times torque
        power_prop = omega_p * torque_prop

        return thrust_prop, torque_prop


    #creates the new motor thrust and torque function, which is an over simplification,
    #but is better for our simpler modelling and figuring everything out
    def _motor_thrust_torque_simplified(self, delta_t: float):

        if delta_t > 1.0:
            delta_t = 1.0
        #returns the delta_t times the max thrust
        return delta_t*CONDA.MaxThrust
    


    def _update_true_state(self):

        pn = self._state.item(0)
        pu = self._state.item(1)

        pn_dot = self._state.item(2)
        pu_dot = self._state.item(3)

        theta = self._state.item(4)
        q = self._state.item(5)

        Va = self._Va

        alpha = self._alpha

        v_air = self.v_air_body

        #calls the update function on the state message
        self.true_state.update(pos_2D=np.array([[pn],[pu]]),
                               vel_2D=np.array([[pn_dot],[pu_dot]]),
                               theta=theta,
                               q=q,
                               v_air=v_air,
                               Va=Va,
                               alpha=alpha)


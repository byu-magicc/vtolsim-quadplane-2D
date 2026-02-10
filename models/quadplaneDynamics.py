import numpy as np
import parameters.anaconda_parameters as CONDA
import parameters.simulation_parameters as SIM
from tools.rotations import *

from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta

from rrt_mavsim.message_types.msg_plane import MsgPlane
from rrt_mavsim.tools.plane_projections_2 import getNormalBasis, map_2D_to_3D, map_3D_to_2D



class QuadplaneDynamics:
    
    #please note that everything in the main state vector is given in the INTERTIAL frame of reference
    #we do this because the velcotiy for the trajectory is given in the inertial frame. same with accel and position
    #we convert back and forth between the two if needed.
    def __init__(self,
                 plane_msg: MsgPlane,
                 ts: float = SIM.ts_simulation,
                 pos_3D_inertial_init: np.ndarray = np.array([[0.0],[0.0],[0.0]]),
                 vel_3D_inertial_init: np.ndarray = np.array([[0.0],[0.0],[0.0]]),
                 theta0: float=CONDA.theta0,
                 q0: float=CONDA.q0):

        pos_2D_init = map_3D_to_2D(vec_3D=pos_3D_inertial_init,
                                   plane=plane_msg)
        vel_2D_init = map_3D_to_2D(vec_3D=vel_3D_inertial_init,
                                   plane=plane_msg)
        
        #saves the plane message
        self.plane_msg = plane_msg
        self.ts = ts

        #creates the initial state array
        self._state = np.concatenate((pos_2D_init, vel_2D_init, np.array([[theta0]]), np.array([[q0]])), axis=0)

        self.true_state = MsgState(plane_msg=plane_msg)

        #calls the function to update the angle of attack, and other important velocities
        self._update_velocity_data()

        #calls the update true state function
        self._update_true_state()


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



    def _f(self,
           state: np.ndarray,
           forces_moments: np.ndarray):

        pn_dot = state.item(CONDA.pn_index)
        pd_dot = state.item(CONDA.pd_index)
        theta = state.item(CONDA.theta_index)
        q = state.item(CONDA.q_index)


        fx_body = forces_moments.item(CONDA.fx_index)
        fz_body = forces_moments.item(CONDA.fz_index)
        My = forces_moments.item(CONDA.My_index)


        R_bodyToInertial = theta_to_rotation_2D(theta=theta)

        #gets the inertial forces;
        f_inertial = R_bodyToInertial @ np.array([[fx_body],[fz_body]])

        pn_ddot = (1.0/CONDA.mass)*f_inertial.item(0)
        pd_ddot = (1.0/CONDA.mass)*f_inertial.item(1)

        theta_dot = q

        q_dot = My/CONDA.Jy

        #gets the derivative vector 
        x_dot = np.array([[pn_dot, pd_dot, pn_ddot, pd_ddot, theta_dot, q_dot]]).T

        return x_dot


    def _forces_moments(self,
                        delta: MsgDelta):

        #gets the rotation matrixes
        theta = self._state.item(CONDA.theta_index)
        R_bodyToInertial = theta_to_rotation_2D(theta=theta)
        R_inertialToBody = R_bodyToInertial.T

        q = self._state.item(CONDA.q_index)

        #obtains the force of gravity in the inertial frame
        fg_inertial_3D = CONDA.mass * CONDA.gravity * CONDA.e3_3D
        #rotates it into the 2D frame
        fg_inertial_2D = map_3D_to_2D(vec_3D=fg_inertial_3D,
                                      plane=self.plane_msg)
        #rotates it into the body frame
        fg_body_2D = R_inertialToBody @ fg_inertial_2D


        #initializes the fx and fy in the body frame with gravity first
        fn_body = fg_body_2D.item(0)
        fd_body = fg_body_2D.item(1)

        #gets the constant coefficients
        #TODO I need to make sure _Va is getting updated
        q_bar = 0.5*CONDA.rho*self._Va**2

        #gets the cosine of alpha
        c_alpha = np.cos(self._alpha)
        s_alpha = np.sin(self._alpha)

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
        F_lift = q_bar * CONDA.S_wing * (CL + CONDA.C_L_q * q_nondim + CONDA.C_L_delta_e * delta.elevator)
        #very important to add the absolute value to the delta_e portion
        F_drag = q_bar * CONDA.S_wing * (CD + CONDA.C_D_q * q_nondim + CONDA.C_D_delta_e * np.abs(delta.elevator))

        #gets the alpha rotation matrix
        R_alpha = alpha_to_rotation_2D(alpha=self._alpha)

        #gets the body frame aerodynamic forces
        f_aero_body = R_alpha @ np.array([[F_drag],[F_lift]])
        fn_aero_body = f_aero_body.item(0)
        fd_aero_body = f_aero_body.item(1)

        #adds to the body frame forces
        fn_body += fn_aero_body
        fd_body += fd_aero_body

        #computes the pitching moment
        My = q_bar * CONDA.S_wing * CONDA.c * (
                CONDA.C_m_0
                + CONDA.C_m_alpha * self._alpha
                + CONDA.C_m_q * q_nondim
                + CONDA.C_m_delta_e * delta.elevator)

        #gets the three thrusts from the simplified (constant*delta) model
        Thrust_front = self._motor_thrust_torque_simplified(delta_t=delta.throttle_front)
        Thrust_rear = self._motor_thrust_torque_simplified(delta_t=delta.throttle_rear)
        Thrust_forward = self._motor_thrust_torque_simplified(delta_t=delta.throttle_thrust)

        #gets the thrust vector
        thrustVector = np.array([[Thrust_front],[Thrust_rear],[Thrust_forward]])
        #gets the body frame thrust forces
        f_thrust_body = CONDA.thrust_forces_mixing @ thrustVector
        fn_thrust_body = f_thrust_body.item(0)
        fd_thrust_body = f_thrust_body.item(1)

        fn_body += fn_thrust_body
        fd_body += fd_thrust_body
        
        M_thrust = CONDA.thrust_moments_mixing @ thrustVector
        
        #and then adds it
        My += M_thrust.item(0)

        forces_moments = np.array([[fn_body],[fd_body],[My]])

        
        return forces_moments

    def _update_true_state(self):
        pn = self._state.item(CONDA.pn_index)
        pd = self._state.item(CONDA.pd_index)

        pn_dot = self._state.item(CONDA.pn_dot_index)
        pd_dot = self._state.item(CONDA.pd_dot_index)

        theta = self._state.item(CONDA.theta_index)
        q = self._state.item(CONDA.q_index)

        Va = self._Va

        alpha = self._alpha
        v_air = self.v_air_body

        #updates the true state
        self.true_state.update(pos_2D=np.array([[pn],[pd]]),
                               vel_2D=np.array([[pn_dot],[pd_dot]]),
                               theta=theta,
                               q=q,
                               v_air=v_air,
                               Va=Va,
                               alpha=alpha)

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
        self._Va = float(np.linalg.norm(self.v_air_body))
        ur = self.v_air_body.item(0)
        wr = self.v_air_body.item(1)
        # compute angle of attack
        if ur==0:
            self._alpha = -np.sign(wr)*np.pi/2.
        else:
            self._alpha = np.arctan2(wr, ur)

        potato = 0

    #creates the new motor thrust and torque function, which is an over simplification,
    #but is better for our simpler modelling and figuring everything out
    def _motor_thrust_torque_simplified(self, delta_t: float):

        if delta_t > 1.0:
            delta_t = 1.0
        #returns the delta_t times the max thrust
        return delta_t*CONDA.MaxThrust

def alpha_to_rotation_2D(alpha: float)->np.ndarray:

    c_alpha = np.cos(alpha)
    s_alpha = np.sin(alpha)

    R_alpha = np.array([[-c_alpha, s_alpha],
                        [-s_alpha, -c_alpha]])


    return R_alpha

#gets the theta to rotation function
def theta_to_rotation_2D(theta: float)->np.ndarray:

    c_theta = np.cos(theta)
    s_theta = np.sin(theta)

    R_bodyToInertial = np.array([[c_theta, s_theta],
                                 [-s_theta, c_theta]])
    return R_bodyToInertial





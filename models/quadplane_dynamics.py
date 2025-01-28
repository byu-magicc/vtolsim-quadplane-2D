#quadplane dynamics
#-this file implements the dynamic equations of motion for the quad plane
#uses unit quaternion for the attitude state
import numpy as np
import parameters.anaconda_parameters as QP
from tools.rotations import rotation_to_euler, euler_to_rotation
from tools.quaternions import *
from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta


#creates the QuadplaneDynamics class
class QuadplaneDynamics:
    #creates the initialization function
    #saves the timestep for simulation, as well as the 
    #bool to enable whether the quadrotors will be enabled or disabled in the dynamics
    def __init__(self, ts: float):
        self._ts = ts
        #creates the state array and initializes them to the original positions
        self._state = np.array([
            [QP.pn0],    # [0]  north position
            [QP.pd0],    # [1]  down position
            [QP.u0],     # [2]  velocity along body x-axis
            [QP.w0],     # [3]  velocity along body z-axis
            [QP.theta0], # [4] initial pitch angle
            [QP.q0],     # [5]  pitch rate
        ])
        self.true_state = MsgState()
        self._update_velocity_data()
        self._forces_moments(delta=MsgDelta())
        # update the message class for the true state
        self._update_true_state()

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
            forces_moments: np.ndarray 
            ):
        '''
            Implements equations of motion xdot = f(x, u)
        '''
        # extract the states
        # pn = state.item(0)
        # pd = state.item(1)
        u = state.item(2)
        w = state.item(3)
        theta = state.item(4)
        # rotation from body to world frame
        R = np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta), np.cos(theta)]]) 
        q = state.item(5)
        #   extract forces/moments in body frame
        fx = forces_moments.item(0)
        fz = forces_moments.item(1)
        My = forces_moments.item(2)
        # position kinematics
        pn_dot = u
        pd_dot = w
        # position dynamics
        f_w = R @ np.array([[fx], [fz]])
        u_dot = f_w.item(0)/QP.mass
        w_dot = f_w.item(1)/QP.mass
        # rotational kinematics
        theta_dot = q
        # rotatonal dynamics
        q_dot = My/QP.Jy
        # collect the derivative of the states
        x_dot = np.array([[pn_dot, pd_dot, u_dot, w_dot, theta_dot, q_dot]]).T
        return x_dot

    #creates the update velocity data function
    def _update_velocity_data(self, wind: np.ndarray=np.zeros((4,1))):
        #gets the steady state wind
        steady_state = wind[0:2]
        gust = wind[2:4]
        # convert wind vector from world to body frame
        theta = self._state.item(4)
        R = np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta), np.cos(theta)]]) # rotation from body to world frame
        wind_body_frame = R.T @ steady_state  # rotate steady state wind to body frame
        wind_body_frame += gust  # add the gust
        self._wind = R @ wind_body_frame  # wind in the world frame
        #gets the velocity in the body frame
        velocity_body_frame = R.T @ self._state[2:4]
        # velocity vector relative to the airmass
        self.v_air = velocity_body_frame - wind_body_frame
        # compute airspeed
        self._Va = np.linalg.norm( self.v_air )
        ur = self.v_air.item(0)
        wr = self.v_air.item(1)
        # compute angle of attack
        if ur==0:
            self._alpha = np.sign(wr)*np.pi/2.
        else:
            self._alpha = np.arctan(wr/ur)
        #computes the groundspeed
        self._Vg = np.linalg.norm(velocity_body_frame)
        
    #creates the forces moments function
    def _forces_moments(self, delta: MsgDelta)->np.ndarray:
        # rotation from body to world frame
        theta = self._state.item(4)
        R = np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta), np.cos(theta)]]) 
        # pitch rate
        q = self._state.item(5)
        # gravitational force in body frame
        f_g = R.T @ np.array([[0.], [QP.mass * QP.gravity]])
        # gets each portion of the gravitational force
        fx = f_g.item(0)
        fz = f_g.item(1)
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
        sigma = (1 + tmp1 + tmp2) / ((1 + tmp1) * (1 + tmp2))
        CL = (1 - sigma) * (QP.C_L_0 + QP.C_L_alpha * self._alpha) \
             + sigma * 2 * np.sign(self._alpha) * sa**2 * ca
        CD = QP.C_D_p + ((QP.C_L_0 + QP.C_L_alpha * self._alpha)**2)/(np.pi * QP.e * QP.AR)
        # compute Lift and Drag Forces
        F_lift = qbar * QP.S_wing * (CL + QP.C_L_q * q_nondim + QP.C_L_delta_e * delta.elevator)
        F_drag = qbar * QP.S_wing * (CD + QP.C_D_q * q_nondim + QP.C_D_delta_e * delta.elevator)
        # compute longitudinal forces in body frame
        fx += - ca * F_drag + sa * F_lift
        fz += - sa * F_drag - ca * F_lift
        # compute pitching moment 
        My = qbar * QP.S_wing * QP.c * (
                QP.C_m_0
                + QP.C_m_alpha * self._alpha
                + QP.C_m_q * q_nondim
                + QP.C_m_delta_e * delta.elevator
        )
        # velocity in world frame
        v_w = np.array([[self._state.item(2)],[self._state.item(3)]])
        # compute velocity in the body frame
        v_b = R.T @ v_w
        # compute airspeed through each propeller
        V_f = -v_b.item(1)
        V_r = -v_b.item(1)
        V_t = -v_b.item(0)
        # compute forces and torques from each propeller
        T_f, Q_f = self._motor_thrust_torque(V_f, delta.throttle_front)
        T_r, Q_r = self._motor_thrust_torque(V_r, delta.throttle_rear)
        T_t, Q_t = self._motor_thrust_torque(V_t, delta.throttle_thrust)
        # add propeller forces and torques to body
        fx += T_t
        fz += -T_f - T_r
        My += QP.ell_f * T_f - QP.ell_r * T_r
        #returns the forces
        return np.array([[fx, fz, My]]).T
    
    def _motor_thrust_torque(self, Vp: float, delta_t: float)->tuple[float, float]:
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
        rho = QP.rho
        #gets the voltage in, based on the delta_t
        V_in = QP.V_max * delta_t
        # Quadratic formula to solve for motor speed
        a = C_Q0 * rho * np.power(D_prop, 5)/((2.*np.pi)**2)
        b = (C_Q1 * rho * np.power(D_prop, 4)/ (2.*np.pi)) * Vp + KQ**2/R_motor
        c = C_Q2 * rho * np.power(D_prop, 3)* Vp**2 - (KQ / R_motor) * V_in + KQ * i0        
        # Consider only positive root
        Omega_p = (-b + np.sqrt(b**2 - 4*a*c)) / (2.*a)
        aa = rho * np.power(D_prop,4) / (4. * np.pi**2)
        bb = rho * np.power(D_prop,3) * Vp/(2 * np.pi)
        cc = rho * D_prop**2 * Vp**2
        T_p = (C_T0 * aa) * Omega_p**2 + (C_T1 * bb) * Omega_p + (C_T2 * cc)
        Q_p = (C_Q0 * D_prop * aa) * Omega_p**2 + (C_Q1 * D_prop * bb) * Omega_p + (C_Q2 * D_prop * cc)
        return T_p, Q_p

    #function to update the true state
    def _update_true_state(self):
        '''update the class structure for the true state '''
        self.true_state.pos = np.array([[self._state.item(0)], [0.0], [self._state.item(1)]])
        self.true_state.vel = np.array([[self._state.item(2)], [0.0], [self._state.item(3)]])
        self.true_state.R = euler_to_rotation(0., self._state.item(4), 0.)
        self.true_state.omega = np.array([[0.], [self._state.item(5)], [0.0]])
        self.true_state.gyro_bias = np.array([
            [0.],
            [0.],
            [0.]])  
        self.true_state.Va = self._Va
        self.true_state.alpha = self._alpha
        self.true_state.beta = 0.
        self.true_state.Vg = self._Vg
        self.true_state.chi = 0.
        self.true_state.v_air = self.v_air

    def _set_internal_state(self, state: MsgState):
        roll, pitch, yaw = rotation_to_euler(state.R)
        self._state = np.array([
            [state.pos.item(0)],    # [0]  north position
            [state.pos.item(1)],    # [1]  down position
            [state.vel.item(0)],     # [2]  velocity along body x-axis
            [state.vel.item(1)],     # [3]  velocity along body z-axis
            [pitch], # [4] initial pitch angle
            [state.omega.item(1)],     # [5]  pitch rate
        ])

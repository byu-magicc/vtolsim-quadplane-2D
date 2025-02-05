"""
    - Last Update:
        1/24/2025  - RWB
        1/29/2025 - RWB
"""
import numpy as np 
from scipy.optimize import minimize
import parameters.anaconda_parameters as QP
from message_types.msg_delta import MsgDelta

from models.quadplane_dynamics import QuadplaneDynamics
from message_types.msg_state import MsgState
from tools.saturate import saturate

# from scipy.linalg import solve_continuous_are, inv
# from tools.rotations import rotation_to_euler, euler_to_rotation
# import parameters.anaconda_parameters as PARAM
# #from controllers.integrator import Integrator
# from message_types.msg_trajectory import MsgTrajectory

class ControlAllocator:
    def __init__(self, ts_control:float):
        self.Ts = ts_control
        self.delta = MsgDelta()

    def update(self, 
               wrench_des: np.ndarray, 
               state: MsgState,
               ):
        Va = state.Va
        alpha = state.alpha
        q = state.omega.item(1)
        F_des_x = wrench_des.item(0)
        F_des_z = wrench_des.item(1)
        M_des = wrench_des.item(2)
        if Va > 0.1:
            q_nondim = q * QP.c / (2 * Va)  
        else:
            q_nondim = 0.0
        qbar = 0.5 * QP.rho * Va**2  
        M_0 = qbar * QP.S_wing * QP.c * (QP.C_m_0 + QP.C_m_alpha * alpha)
        M_q = qbar * QP.S_wing * QP.c * QP.C_m_q
        M_delta_e = qbar * QP.S_wing * QP.c * QP.C_m_q
        # use elevator to get as much torque as possible
        self.delta.elevator = saturate((M_des - M_0 - M_q * q_nondim) / (M_delta_e+0.00001), 
                                  -1, 1)
        M_unachieved = M_des - M_0 - M_q * q_nondim - M_delta_e * self.delta.elevator
        # compute desired thrust for each rotor
        T_f_des = -QP.ell_r/(QP.ell_r+QP.ell_f) * F_des_z + 1/(QP.ell_r+QP.ell_f) * M_unachieved
        T_r_des = QP.ell_f/(QP.ell_r+QP.ell_f) * F_des_z - 1/(QP.ell_r+QP.ell_f) * M_unachieved
        T_t_des = F_des_x
        # compute rotor commands
        # velocity in world frame
        # compute velocity in the body frame
        v_b = state.R.T @ state.vel
        # compute airspeed through each propeller
        V_f = -v_b.item(2)
        V_r = -v_b.item(2)
        V_t = v_b.item(0)
        self.delta.throttle_front = invert_motor(T_f_des, V_f)
        self.delta.throttle_rear = invert_motor(T_r_des, V_r)
        self.delta.throttle_thrust = invert_motor(T_t_des, V_t)
        return self.delta

def invert_motor(T_des: float, Vp: float)->float:
    delta0 = 0.5 # initial guess for throttle
    # define inequality constraints
    cons = ([
        {'type': 'ineq', #>0
            'fun': lambda delta: np.array([# elevator constraint
                1.-delta[0], # delta <=1 
                delta[0], # delta>=0
                ]),
                'jac': lambda delta: np.array([
                [-1.],
                [1.],
                ])
            }
        ])
    result = minimize(objective_motor, 
                      delta0, 
                      method='SLSQP', 
                      args = (T_des, Vp),
                      constraints=cons, 
                      options={'ftol': 1e-10, 'disp': True, 'maxiter': 1000, 'iprint': 0})
    delta = saturate(result.x.item(0), 0., 1.)
    return delta

# objective functions to be minimized
def objective_motor(delta, T_des, Vp):
    T_motor = motor_thrust(Vp, delta)
    J = (T_des-T_motor)**2.0
    return J

def motor_thrust(Vp: float, delta_t: float)->float:
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
    aa = C_T0 * rho * np.power(D_prop,4) / (4. * np.pi**2)
    bb = C_T1 * rho * np.power(D_prop,3) * Vp/(2 * np.pi)
    cc = C_T2 * rho * D_prop**2 * Vp**2
    T_p = (aa) * Omega_p**2 + (bb) * Omega_p + (cc)
    return T_p


"""
    - Last Update:
        1/24/2025  - RWB
        1/29/2025 - RWB
"""
import numpy as np 
from scipy.linalg import solve_continuous_are, inv
from scipy.optimize import minimize
from tools.rotations import rotation_to_euler, euler_to_rotation
from tools.filters import BetaFilter
import parameters.anaconda_parameters as QP
#from controllers.integrator import Integrator
from message_types.msg_state import MsgState
from message_types.msg_trajectory import MsgTrajectory


class TrajectoryTracker:
    def __init__(self, ts_control:float):
        self.Ts = ts_control
        # initialize integrators and delay variables
        A = np.array([
            [0., 0., 1., 0.],
            [0., 0., 0., 1.],
            [0., 0., 0., 0.],
            [0., 0., 0., 0.],
        ])
        B = np.array([
            [0., 0.],
            [0., 0.],
            [1., 0.],
            [0., 1.],
        ])
        Q = 0.001 * np.diag([
            1., # pn - error
            1., # pd - error
            1., # v_n - error
            1., # v_e - error
            ]) 
        R = 1000 * np.diag([
            1., # u - Fn
            1., # u - Fe
            ])  
        P = solve_continuous_are(A, B, Q, R)
        self.K = inv(R) @ B.T @ P
        self.kp_theta = 1.
        self.kd_theta = 10.
        self.dirty_derivative_of_theta = BetaFilter(beta=0.1,Ts=ts_control)
        self.dirty_derivative_of_theta_dot = BetaFilter(beta=0.1,Ts=ts_control)
        self.theta_star = 0.  # theta_star delayed by one sample
        self.commanded_state = MsgState()

    def update(self, 
               trajectory: MsgTrajectory, 
               state: MsgState,
               ):
        # compute error state for LQR trajectory tracker
        x_err = np.array([
            [state.pos.item(0) - trajectory.pos.item(0)],
            [state.pos.item(2) - trajectory.pos.item(1)],
            [state.vel.item(0) - trajectory.vel.item(0)],
            [state.vel.item(2) - trajectory.vel.item(1)],
            ])       
        # compute desired inertial frame force using LQR
        e_z = np.array([[0.], [1.]])
        F_des_i = QP.mass * (trajectory.accel - QP.gravity * e_z - self.K @ x_err)
        # compute optima pitch and associated desired propeller force in body frame
        self.theta_star, F_des_b = optimize_pitch(F_des_i, trajectory, self.theta_star, self.Ts)
        # differentiate theta_star using beta-filter
        theta_star_dot = self.dirty_derivative_of_theta.update(self.theta_star)
        theta_star_ddot = self.dirty_derivative_of_theta_dot.update(theta_star_dot)
        # desired moment
        phi, theta, psi = rotation_to_euler(state.R)
        q = state.omega.item(1)
        M_des = QP.Jy * (theta_star_ddot \
                         - self.kd_theta * (q - theta_star_dot) \
                            - self.kp_theta * (theta-self.theta_star))
        # desired wrench
        W_des = np.concatenate((F_des_b, np.array([[M_des]])))
        # update delayed variables
        # construct control outputs and commanded states
        self.commanded_state.pos = np.array([[trajectory.pos.item(0)], 
                                             [0.], 
                                             [trajectory.pos.item(1)]])
        self.commanded_state.vel = np.array([[trajectory.vel.item(0)], 
                                             [0.], 
                                             [trajectory.vel.item(1)]])
        self.commanded_state.R = euler_to_rotation(0., self.theta_star, 0.)
        return W_des, self.commanded_state


# compute the pitch angle that minimize thrusters
def optimize_pitch(F_des_i: np.ndarray,
                   trajectory: MsgTrajectory,
                   theta_old: float,
                   Ts: float,
                   )->float:
    # define limits on pitch theta and pitchrate q
    theta_max = np.radians(25)
    theta_min = -np.radians(25)
    q_max = np.radians(1)
    # compute speed and flight path angle along trajectory
    V_traj = np.linalg.norm(trajectory.vel)
    gamma_traj = np.arctan2(-trajectory.vel.item(1), trajectory.vel.item(0))
    # compute optimal pitch
    theta0 = theta_old # start optimization algorithm at old theta
    # define inequality constraints
    cons = ([
        {'type': 'ineq', #>=0
            'fun': lambda theta: np.array([
                theta[0]-theta_min,
                theta_max-theta[0],
                theta[0]-theta_old+q_max*Ts,
                q_max*Ts + theta_old - theta[0],
                ]),
            # 'jac': lambda theta: np.array([
            #     [1.],
            #     [-1.],
            #     [1.],
            #     [-1.],
            #     ])
        }
    ])
    result = minimize(objective_pitch, 
                        theta0, 
                        method='SLSQP', 
                        args = (F_des_i, V_traj, gamma_traj),
                        constraints=cons, 
                        options={'ftol': 1e-10, 'disp': True, 'maxiter': 1000, 'iprint': 0})
    theta_star = result.x.item(0)
    F_des_b = F_p(theta_star, F_des_i, V_traj, gamma_traj)
    return theta_star, F_des_b

# objective functions to be minimized
def objective_pitch(theta, F_des_i, V_traj, gamma_traj):
    # desired force from propellers
    Fp = F_p(theta.item(0), F_des_i, V_traj, gamma_traj)
    if np.abs(V_traj)<5.:
        J = np.abs(Fp.item(1))**2
    else:
    # objective is the 1-norm of Fp
        J = np.abs(Fp.item(0)) + np.abs(Fp.item(1))
    return J

# compute the desired force from the propellers
def F_p(theta, F_des, V_traj, gamma_traj):
    # rotation from inertial to body frame
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    # desired force from propellers in body frame
    Fp = R @ F_des - F_0(V_traj, theta - gamma_traj)
    return Fp

# aerodynamic forces in body frame
def F_0(Va, alpha):
    R = np.array([[np.cos(alpha), -np.sin(alpha)], [np.sin(alpha), np.cos(alpha)]])
    # compute Lift and Drag coefficients
    tmp1 = np.exp(-QP.M * (alpha - QP.alpha0))
    tmp2 = np.exp(QP.M * (alpha + QP.alpha0))
    sigma = (1 + tmp1 + tmp2) / ((1 + tmp1) * (1 + tmp2))
    CL = (1 - sigma) * (QP.C_L_0 + QP.C_L_alpha * alpha) \
        + sigma * 2 * np.sign(alpha) * np.sin(alpha)**2 * np.cos(alpha)
    CD = QP.C_D_p + ((QP.C_L_0 + QP.C_L_alpha * alpha)**2)/(np.pi * QP.e * QP.AR)
    F = 0.5 * QP.rho * Va**2 * QP.S_wing * R @ np.array([[-CD], [-CL]])
    return F
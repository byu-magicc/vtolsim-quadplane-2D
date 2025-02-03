"""
    - Last Update:
        1/24/2025  - RWB
        1/29/2025 - RWB
"""
import numpy as np 
from scipy.linalg import solve_continuous_are, inv
from tools.rotations import rotation_to_euler, euler_to_rotation, theta_to_rotation_2d
import parameters.anaconda_parameters as PARAM
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
        Q = 1e-1 * np.diag([
            1., # pn - error
            1., # pd - error
            1., # v_n - error
            1., # v_e - error
            ]) 
        R = 1 * np.diag([
            1., # u - Fn
            1., # u - Fe
            ])  
        P = solve_continuous_are(A, B, Q, R)
        self.K = inv(R) @ B.T @ P
        self.kp_theta = 1.
        self.kd_theta = 1.
        self.theta_star_d1 = 0.  # theta star delayed one sample
        self.theta_star_dot = 0.  # first derivative of theta star
        self.theta_star_dot_d1 = 0.  # first derivative of theta star delayed by one sample
        self.theta_star_ddot = 0.  # second derivative of theta star
        self.beta = 0.5  # coefficient for beta filter (dirty derivative)
        self.commanded_state = MsgState()

    def update(self, 
               trajectory: MsgTrajectory, 
               state: MsgState):
        

        roll, pitch, yaw = rotation_to_euler(state.R)
        q = state.omega.item(1)
        x_err = np.array([
            [state.pos.item(0) - trajectory.pos.item(0)],
            [state.pos.item(2) - trajectory.pos.item(1)],
            [state.vel.item(0) - trajectory.vel.item(0)],
            [state.vel.item(2) - trajectory.vel.item(1)],
            ])       
        R = theta_to_rotation_2d(theta=pitch)
        
        e_z = np.array([[0.], [1.]])
        F_des_i = QP.mass * (trajectory.accel - QP.gravity * e_z - self.K @ x_err)
        # compute optima pitch and associated desired propeller force in body frame
        theta_star, F_des_b = optimize_pitch(F_des_i, trajectory, self.theta_star_d1, self.Ts)
        # differentiate theta_star using beta-filter
        self.theta_star_dot = self.beta * self.theta_star_dot \
            + (1 - self.beta) * (theta_star - self.theta_star_d1) / self.Ts
        self.theta_star_ddot = self.beta * self.theta_star_ddot \
            + (1 - self.beta) * (self.theta_star_dot - self.theta_star_dot_d1)/self.Ts
        # desired moment
        phi, theta, psi = rotation_to_euler(state.R)
        q = state.omega.item(1)
        M_des = QP.Jy * (self.theta_star_ddot \
                         - self.kd_theta * (q - self.theta_star_dot) \
                            - self.kp_theta * (theta-theta_star))
        # desired wrench
        W_des = np.concatenate((F_des_b, np.array([[M_des]])))
        # update delayed variables
        self.theta_star_d1 = theta_star
        self.theta_star_dot_d1 = self.theta_star_dot
        # construct control outputs and commanded states
        self.commanded_state.pos = np.array([[trajectory.pos.item(0)], 
                                             [0.], 
                                             [trajectory.pos.item(1)]])
        self.commanded_state.vel = np.array([[trajectory.vel.item(0)], 
                                             [0.], 
                                             [trajectory.vel.item(1)]])
        self.commanded_state.R = euler_to_rotation(0., theta_star, 0.)
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
            'jac': lambda theta: np.array([
                [1.],
                [-1.],
                [1.],
                [-1.],
                ])
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
    # objective is the 1-norm of Fp
    J = np.abs(Fp.item(0)) + np.abs(Fp.item(1))
    return J

# compute the desired force from the propellers
def F_p(theta, F_des, V_traj, gamma_traj):
    # rotation from inertial to body frame
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    # desired force from propellers
    Fp = R @ F_des - F_0(V_traj, theta - gamma_traj)
    return Fp

# aerodynamic forces in body frame
def F_0(Va, alpha):
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    R = np.array([[ca, -sa], [sa, ca]])
    # compute Lift and Drag coefficients
    tmp1 = np.exp(-QP.M * (alpha - QP.alpha0))
    tmp2 = np.exp(QP.M * (alpha + QP.alpha0))
    sigma = (1 + tmp1 + tmp2) / ((1 + tmp1) * (1 + tmp2))
    CL = (1 - sigma) * (QP.C_L_0 + QP.C_L_alpha * alpha) \
        + sigma * 2 * np.sign(alpha) * sa**2 * ca
    CD = QP.C_D_p + ((QP.C_L_0 + QP.C_L_alpha * alpha)**2)/(np.pi * QP.e * QP.AR)
    F = 0.5 * QP.rho * Va**2 * QP.S_wing * R @ np.array([[-CD], [-CL]])
    return F
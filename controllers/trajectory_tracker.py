"""
    - Last Update:
        1/24/2025  - RWB
"""
import numpy as np 
from scipy.linalg import solve_continuous_are, inv
from tools.rotations import rotation_to_euler, euler_to_rotation
import parameters.anaconda_parameters as PARAM
#from controllers.integrator import Integrator
from message_types.msg_state import MsgState
from message_types.msg_trajectory import MsgTrajectory


class TrajectoryTracker:
    def __init__(self, ts_control:float):
        self.Ts = ts_control
        # initialize integrators and delay variables
        A = np.array([
            [0., 0., 1., 0., 0., 0.],
            [0., 0., 0., 1., 0., 0.],
            [0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 1.],
            [0., 0., 0., 0., 0., 0.],
        ])
        B = np.array([
            [0., 0., 0.],
            [0., 0., 0.],
            [1., 0., 0.],
            [0., 1., 0.],
            [0., 0., 0.],
            [0., 0., 1.],
        ])
        Q = 1e-5 * np.diag([
            1., # pn - error
            1., # pd - error
            1., # v_n - error
            1., # v_e - error
            np.radians(1), # theta - error
            1., # q - error 
            ]) 
        R = 1000 * np.diag([
            1., # u - Fn
            1., # u - Fe
            1., # u - M
            ])  
        P = solve_continuous_are(A, B, Q, R)
        K = inv(R) @ B.T @ P
        self.K_F = K[0:2,:]
        self.K_M = K[2:3,:]
        self.commanded_state = MsgState()

    def update(self, 
               trajectory: MsgTrajectory, 
               state: MsgState,
               ):
        roll, pitch, yaw = rotation_to_euler(state.R)
        q = state.omega.item(1)
        x_err = np.array([
            [state.pos.item(0) - trajectory.pos.item(0)],
            [state.pos.item(2) - trajectory.pos.item(1)],
            [state.vel.item(0) - trajectory.vel.item(0)],
            [state.vel.item(2) - trajectory.vel.item(1)],
            [pitch - trajectory.pitch],
            [q-trajectory.pitch_rate],
            ])       
        R = np.array([[np.cos(pitch), -np.sin(pitch)], [np.sin(pitch), np.cos(pitch)]])
        e_z = np.array([[0.], [1.]])
        F_des = PARAM.mass * R.T @ (trajectory.accel - PARAM.gravity * e_z - self.K_F @ x_err)
        M_des = PARAM.Jy * (trajectory.pitch_accel - self.K_M @ x_err)
        W_des = np.concatenate((F_des, M_des), axis=0)
        # construct control outputs and commanded states
        self.commanded_state.pos = np.array([[trajectory.pos.item(0)], 
                                             [0.], 
                                             [trajectory.pos.item(1)]])
        self.commanded_state.vel = np.array([[trajectory.vel.item(0)], 
                                             [0.], 
                                             [trajectory.vel.item(1)]])
        self.commanded_state.R = euler_to_rotation(phi=0., theta=trajectory.pitch, psi=0.)
        self.commanded_state.omega = np.array([[0.], [trajectory.pitch_rate], [0.]]) 
        return W_des, self.commanded_state

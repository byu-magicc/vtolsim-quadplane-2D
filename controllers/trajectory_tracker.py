"""
autopilot block for mavsim_python
    - Beard & McLain, PUP, 2012
    - Last Update:
        1/23/2025  - RWB
"""
import numpy as np # import array, sin, cos, radians, concatenate, zeros, diag
from scipy.linalg import solve_continuous_are, inv
#from models.ss_model_Va_0 import Va, trim_state, trim_input, A, B
from tools.rotations import rotation_to_euler
#from controllers.integrator import Integrator

import models.model_coef as M
from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta
from tools.transfer_function import TransferFunction
from tools.wrap import wrap

def saturate(input, low_limit, up_limit):
    if input <= low_limit:
        output = low_limit
    elif input >= up_limit:
        output = up_limit
    else:
        output = input
    return output


class TrajTracker:
    def __init__(self, ts_control):
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
        Q = np.diag([
            1., # pn - error
            1., # pd - error
            1., # v_n - error
            1., # v_e - error
            np.radians(1), # theta - error
            1., # q - error 
            ]) 
        R = np.diag([
            1., # u - Fn
            1., # u - Fe
            1., # u - M
            ])  
        P = solve_continuous_are(A, B, Q, R)
        K = inv(R) @ B.T @ P
        self.K_F = K[0:2,:]
        self.K_M = K[2,:]
        self.commanded_state = MsgState()

    def update(self, trajectory, state):
        roll, pitch, yaw = rotation_to_euler(state.R)
        q = state.omega.item(1)
        x_err = np.array([
            [state.pos.item(0) - trajectory.position.item(0)],
            [state.pos.item(2) - trajectory.position.item(1)],
            [state.vel.item(0) - trajectory.velocity.item(0)],
            [state.vel.item(2) - trajectory.velocity.item(1)],
            [pitch - trajectory.pitch],
            [q-trajectory.pitch_rate],
            ])       
        R = np.array([[np.cos(pitch), -np.sin(pitch)], [np.sin(pitch), np.cos(pitch)]])
        e_z = np.array([[0.], [0.], [1.]])
        F_des = M.mass * R.T @ (trajectory.acceleration - M.gravity * e_z - self.K_F @ x_err)
        M_des = M.J_y * (trajectory.pitch_accel - self.K_M @ x_err)
        W_des = np.concatenate((F_des, M_des), axis=0)


        # construct control outputs and commanded states
        self.commanded_state.altitude = cmd.altitude_command
        self.commanded_state.Va = cmd.airspeed_command
        self.commanded_state.phi = 0
        self.commanded_state.theta = 0
        self.commanded_state.chi = cmd.course_command
        return W_des, self.commanded_state

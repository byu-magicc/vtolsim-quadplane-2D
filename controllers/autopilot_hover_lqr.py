"""
autopilot block for mavsim_python
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/10/22 - RWB
        7/13/2023 - RWB
"""
import numpy as np # import array, sin, cos, radians, concatenate, zeros, diag
from scipy.linalg import solve_continuous_are, inv
from models.ss_model_Va_0 import Va, trim_state, trim_input, A, B
from tools.rotations import rotation_to_euler
from controllers.integrator import Integrator

import parameters.control_parameters as AP
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


class Autopilot:
    def __init__(self, ts_control):
        self.Ts = ts_control
        # initialize integrators and delay variables
        self.integrator_north = 0.
        self.integrator_altitude = 0.
        self.error_north_d1 = 0.
        self.error_altitude_d1 = 0.
        Cr = np.array([[1., 0., 0., 0., 0., 0.], [0., -1., 0., 0., 0., 0.]])
        AA = np.concatenate((
                np.concatenate((A, np.zeros((6,2))), axis=1),
                np.concatenate((Cr, np.zeros((2,2))), axis=1)),
                    axis=0)
        BB = np.concatenate((B, np.zeros((2, 2))), axis=0)
        Q = np.diag([
            1., # pn
            1., # pd
            0.001, # u
            0.001, # w
            0.001, # theta
            0.001, # q
            10.,  # int_pn
            10.,  # int_h
            ]) 
        R = np.diag([
            1000., # elevator
            .001, # throttle_front
            .001, # throttle_rear
            1000., # throttle_thrust
            ])  
        P = solve_continuous_are(AA, BB, Q, R)
        self.K = inv(R) @ BB.T @ P
        self.commanded_state = MsgState()

    def update(self, trajectory, state):
        roll, pitch, yaw = rotation_to_euler(state.R)
        q = state.omega.item(1)
        x_err = np.array([
            [state.pos.item(0) - trajectory.position.item(0)],
            [state.pos.item(2) - trajectory.position.item(1)],
            [state.pos.item(3) - trajectory.velocity.item(0)],
            [state.pos.item(5) - trajectory.velocity.item(1)],
            [pitch - trajectory.pitch],
            [q-trajectory.pitch_rate],
            [self.integrator_north],
            [self.integrator_altitude],
            ])       
        # update integrators 
        self.integrator_north \
            += (self.Ts/2) * (x_err.item(0) + self.error_north_d1)
        self.error_north_d1 = x_err.item(0)
        self.integrator_altitude \
            += (self.Ts/2) * (x_err.item(1) + self.error_altitude_d1)
        self.error_altitude_d1 = x_err.item(1)
        tmp = -self.K @ x_err
        delta_e = saturate(tmp.item(0), -radians(30), radians(30))
        delta_t = saturate(tmp.item(1), 0.0, 1.0)

        # construct control outputs and commanded states
        delta = MsgDelta(elevator=delta_e,
                         aileron=delta_a,
                         rudder=delta_r,
                         throttle=delta_t)
        self.commanded_state.altitude = cmd.altitude_command
        self.commanded_state.Va = cmd.airspeed_command
        self.commanded_state.phi = 0
        self.commanded_state.theta = 0
        self.commanded_state.chi = cmd.course_command
        return delta, self.commanded_state

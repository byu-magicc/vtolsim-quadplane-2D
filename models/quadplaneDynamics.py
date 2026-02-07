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

        #creates the initial state array
        self._state = np.concatenate((pos_2D_init, vel_2D_init, np.array([[theta0]]), np.array([[q0]])), axis=0)

        self.true_state = MsgState(plane_msg=plane_msg)

        #calls the function to update the angle of attack, and other important velocities
        self._update_velocity_data()


    def _update_true_state(self):
        pn = self._state.item(0)
        pd = self._state.item(1)

        pn_dot = self._state.item(2)
        pd_dot = self._state.item(3)

        theta = self._state.item(4)
        q = self._state.item(5)

        Va = self._Va

        alpha = self._alpha
        v_air = self.v_air_body

        #updates the true state
        self.true_state.update(pos_2D=np.array([[pn],[pd]]),
                               )


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
            self._alpha = np.arctan2(wr, ur)

        potato = 0


#gets the theta to rotation function
def theta_to_rotation_2D(theta: float)->np.ndarray:

    c_theta = np.cos(theta)
    s_theta = np.sin(theta)

    R_bodyToInertial = np.array([[c_theta, s_theta],
                                 [-s_theta, c_theta]])
    return R_bodyToInertial




import numpy as np
from tools.old.rotations import rotation_to_euler, rotation_to_theta_2d, theta_to_rotation_2d
import scipy as sp
from rrt_mavsim.message_types.msg_plane import MsgPlane
from rrt_mavsim.tools.plane_projections_2 import map_2D_to_3D, map_3D_to_2D


#NOTE: Notice that this message class is all based in a 2D environment. In a 2d world, this is where it operates. 
#Everything for 3D is done outside of this space.
class MsgState:

    def __init__(self,
                 plane_msg: MsgPlane,
                 pos_2D: np.ndarray=np.array([[0.],[0.]]),
                 vel_2D: np.ndarray=np.array([[0.],[0.]]),
                 theta: float = 0.0,
                 q: float = 0.0,
                 v_air: np.ndarray = np.array([[0.0],[0.0]]),
                 Va: float = 25.0,
                 alpha: float = 0.0):
        
        self.plane_msg = plane_msg
        
        #calls the update function
        self.update(pos_2D=pos_2D,
                    vel_2D=vel_2D,
                    theta=theta,
                    q=q,
                    v_air=v_air,
                    Va=Va,
                    alpha=alpha)


    def update(self,
               pos_2D: np.ndarray=np.array([[0.],[0.]]),
               vel_2D: np.ndarray=np.array([[0.],[0.]]),
               theta: float = 0.0,
               q: float = 0.0,
               v_air: np.ndarray = np.array([[0.0],[0.0]]),
               Va: float = 25.0,
               alpha: float = 0.0):

        self.pos_2D = pos_2D
        self.vel_2D = vel_2D

        self.theta = theta
        self.q = q

        self.v_air = v_air

        self.Va = Va


        self.alpha = alpha

        self.R = theta_to_rotation_2d(theta=theta)

        self.pos_3D = map_2D_to_3D(vec_2D=self.pos_2D,
                                   plane=self.plane_msg)
        self.vel_3D = map_2D_to_3D(vec_2D=self.vel_2D,
                                   plane=self.plane_msg)

        testPoint = 0


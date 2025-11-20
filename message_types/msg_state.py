


import numpy as np
from tools.old.rotations import rotation_to_euler, rotation_to_theta_2d, theta_to_rotation_2d
import scipy as sp


#NOTE: Notice that this message class is all based in a 2D environment. In a 2d world, this is where it operates. 
#Everything for 3D is done outside of this space.
class MsgState:

    def __init__(self,
                 pos: np.ndarray=np.array([[0.],[0.]]),
                 vel: np.ndarray=np.array([[0.],[0.]]),
                 theta: float = 0.0,
                 q: float = 0.0,
                 v_air: np.ndarray = np.array([[0.0],[0.0]]),
                 Va: float = 25.0,
                 alpha: float = 0.0):
        
        #calls the update function
        self.update(pos=pos,
                    vel=vel,
                    theta=theta,
                    q=q,
                    v_air=v_air,
                    Va=Va,
                    alpha=alpha)


    def update(self,
               pos: np.ndarray=np.array([[0.],[0.]]),
               vel: np.ndarray=np.array([[0.],[0.]]),
               theta: float = 0.0,
               q: float = 0.0,
               v_air: np.ndarray = np.array([[0.0],[0.0]]),
               Va: float = 25.0,
               alpha: float = 0.0):

        self.pos = pos
        self.vel = vel

        self.theta = theta
        self.q = q

        self.v_air = v_air

        self.Va = Va
        self.alpha = alpha

        self.R = theta_to_rotation_2d(theta=theta)
"""
msgState 
    - messages type for state, that will be passed between blocks in the architecture
    - Update history:  
        5/3/2021 - RWB
        6/8/2021 - RWB
        4/20/2022 - RWB
        11/16/2023 - RWB
"""
import numpy as np
from tools.rotations import rotation_to_euler, euler_to_rotation
from tools.rotations import theta_to_rotation_2d

class MsgState:

    def __init__(self, 
                 pos: np.ndarray=np.array([[0.], [0.]]), #saves the inertial north and down positions
                 vel: np.ndarray=np.array([[0.], [0.]]), #saves the body frame forward and down velocities
                 R: np.ndarray=np.identity(2), #saves the 2x2 Rotation matrix, which is made by theta
                 omega: np.ndarray=np.array([[0.]]), #saves the q_dot matrix
                 Va: float=0., #saves the airspeed variable
                 alpha: float=0., #saves the angle of attack.
                 ):
            self.pos = pos  
            self.vel = vel  
            self.R = R  
            self.omega = omega  
            self.Va = Va
            self.alpha = alpha

    def add_to_position(self, n=0, e=0, d=0):
        self.pos = self.pos + np.array([[n], [d]])

    
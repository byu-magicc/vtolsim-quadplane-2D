"""
msgState 
    - messages type for state, that will be passed between blocks in the architecture
    - Update history:  
        5/3/2021 - RWB
        6/8/2021 - RWB
        4/20/2022 - RWB
        11/16/2023 - RWB
"""

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

class MsgState:
    def __init__(self, 
                 pos: np.ndarray=np.array([[0.], [0.], [0.]]), #inertial position. east term is always zero
                 vel: np.ndarray=np.array([[0.], [0.], [0.]]), #inertial velocities. east term is always zero
                 R: np.ndarray=np.identity(3), 
                 omega: np.ndarray=np.array([[0.], [0.], [0.]]), 
                 gyro_bias: np.ndarray=np.array([[0.], [0.], [0.]]), 
                 motor_angle: np.ndarray=np.array([[0.], [0.]]),  
                 Va: float=0.,
                 v_air: np.ndarray=np.array([[0.0], [0.0]]), #saves the airspeed vector 
                 alpha: float=0.,  
                 beta: float=0.,  
                 ):
            self.pos = pos  
            self.vel = vel  
            self.R = R  
            self.omega = omega  
            self.gyro_bias = gyro_bias
            self.motor_angle = motor_angle
            self.Va = Va
            self.v_air = v_air
            self.alpha = alpha
            self.beta = beta

    def add_to_position(self, n=0, e=0, d=0):
        self.pos = self.pos + np.array([[n], [e], [d]])

    def euler_angles(self)->tuple[float, float, float]:
        phi, theta, psi = rotation_to_euler(self.R)
        return phi, theta, psi



    
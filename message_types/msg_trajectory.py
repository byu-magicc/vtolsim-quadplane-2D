"""
    - Update history:
        7/1/2021 - RWB
        6/21/2022 - RWB
        4/8/2024 - RWB
        1/20/2025 - RWB
"""
import numpy as np


class MsgTrajectory:
    '''
        Trajectory message, defining the instanteous position, velocity, acceleration, and pitch angle and pitch rate
    '''
    def __init__(self, 
                 position: np.ndarray=np.array([[0.], [0.]]),
                 velocity: np.ndarray=np.array([[0.], [0.]]),
                 acceleration: np.ndarray=np.array([[0.], [0.]]),
                 pitch: float=0.,
                 pitch_rate: float=0.
                 ):
        self.position = position
        self.velocity = velocity
        self.acceleration = acceleration
        self.pitch = pitch
        self.pitch_rate = pitch_rate

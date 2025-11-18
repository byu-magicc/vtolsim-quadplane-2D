"""
    - Update history:
        7/1/2021 - RWB
        6/21/2022 - RWB
        4/8/2024 - RWB
        1/20/2025 - RWB
"""
import numpy as np

#this is the desired  trajectory. the desired state

class MsgTrajectory:
    '''
        Trajectory message, defining the instanteous position, velocity, acceleration, and pitch angle and pitch rate
    '''
    def __init__(self, 
                 pos: np.ndarray=np.array([[0.], [0.]]),
                 vel: np.ndarray=np.array([[0.], [0.]]),
                 accel: np.ndarray=np.array([[0.], [0.]]),
                 pitch: float=0.,
                 pitch_rate: float=0.,
                 pitch_accel: float=0.,
                 ):

        self.update(pos=pos,
                    vel=vel,
                    accel=accel,
                    pitch=pitch,
                    pitch_rate=pitch_rate,
                    pitch_accel=pitch_accel)

    def update(self,
               pos: np.ndarray=np.array([[0.], [0.]]),
               vel: np.ndarray=np.array([[0.], [0.]]),
               accel: np.ndarray=np.array([[0.], [0.]]),
               pitch: float=0.,
               pitch_rate: float=0.,
               pitch_accel: float=0.,):
        
        self.pos = pos
        self.vel = vel
        self.accel = accel
        self.pitch = pitch
        self.pitch_rate = pitch_rate
        self.pitch_accel = pitch_accel
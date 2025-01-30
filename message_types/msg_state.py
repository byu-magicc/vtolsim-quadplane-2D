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
    '''
    Message class that defines the state of the aircraft

    Attributes
    ----------
    pos : np.ndarray (3x1)
        inertial NED position in meters
    vel : np.ndarray (3x1)
        inertial velocity in body frame in m/s
    R : np.ndarray (3x1)
        rotation matrix, body to inertial
    omega : np.ndarray (3x1)
        angular velocity in body frame in rad/sec
    gyro_bias : np.ndarray (3x1)
        gyro bias in rad/sec
    motor_angle : np.ndarray (2x1)
        right/left angles of motors in rad
    Va : float
        airspeed in m/s
    alpha : float
        angle of attach in rad
    beta : float
        sideslip angle in rad
    
    Methods
    -------
    add_to_position(n, e, d) :
        add (n,e,d) to the current position
    euler_angles() :
        returns the euler angles phi, theta, psi
    
    TO DO:  add these
    __add__(self, other)
        Overload the addition '+' operator
    __sub__(self, other):
        Overload the subtraction '-' operator
    __rmul__(self, other: float):
        Overload right multiply by a scalar
    '''
    def __init__(self, 
                 pos: np.ndarray=np.array([[0.], [0.], [0.]]), 
                 vel: np.ndarray=np.array([[0.], [0.], [0.]]), 
                 R: np.ndarray=np.identity(3), 
                 omega: np.ndarray=np.array([[0.], [0.], [0.]]), 
                 gyro_bias: np.ndarray=np.array([[0.], [0.], [0.]]), 
                 motor_angle: np.ndarray=np.array([[0.], [0.]]),  
                 Va: float=0.,
                 v_air = np.array([[0.0],[0.0]]), 
                 alpha: float=0.,  
                 beta: float=0.,  
                 Vg: float = 0,
                 chi: float = 0
                 ):
            self.pos = pos  
            self.vel = vel  
            self.R = R  
            self.omega = omega  
            self.gyro_bias = gyro_bias
            self.motor_angle = motor_angle
            self.Va = Va
            self.alpha = alpha
            self.beta = beta
            self.v_air = v_air
            self.Vg = Vg
            self.chi = chi

    def add_to_position(self, n=0, e=0, d=0):
        self.pos = self.pos + np.array([[n], [e], [d]])

    def euler_angles(self)->tuple[float, float, float]:
        phi, theta, psi = rotation_to_euler(self.R)
        return phi, theta, psi
    
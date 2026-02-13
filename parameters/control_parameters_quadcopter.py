"""
quadSim
    - Chapter 14 assignment for Beard & McLain, PUP, 2012
    - Update history:
        7/7/2021 - RWB
        4/26/2022 - RWB
        7/13/2023 - RWB
"""
import numpy as np
import parameters.anaconda_parameters as QUAD

gravity = QUAD.gravity  # gravity constant
mass = QUAD.mass
rho = QUAD.rho

# ----------saturation limits-------------
xy_torque_limit = 200.0
z_torque_limit = 100.0
pitch_angle_limit = np.radians(30)


#the bounds for north postion and altitude
north_max = 100000
down_max = 100000


# ----------pitch loop-------------
wn_pitch = 10.0
zeta_pitch = 0.707
pitch_kp = QUAD.Jy * wn_pitch**2
pitch_kd = QUAD.Jy * 2 * zeta_pitch * wn_pitch




# ----------north loop-------------
wn_north = 0.5
zeta_north = 0.707
north_kp = wn_north**2
north_kd = 2 * zeta_north * wn_north
north_ki = 0.05


# ----------down loop-------------
wn_down = 2.0 * wn_north
zeta_down = 0.707
down_kp = wn_down**2
down_kd = 2 * zeta_down * wn_down
down_ki = 0.01

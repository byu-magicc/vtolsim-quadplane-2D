import numpy as np
import models.model_coef as TF
import parameters.anaconda_parameters as QUAD

gravity = QUAD.gravity  # gravity constant
rho = QUAD.rho  # density of air
sigma = 0.05  # low pass filter gain for derivative
Va0 = TF.Va_trim

#----------pitch loop-------------
wn_pitch = 15.0 #24.0
zeta_pitch = 0.707
pitch_kp = (wn_pitch**2 - TF.a_theta2) / TF.a_theta3
pitch_kd = (2.0 * zeta_pitch * wn_pitch - TF.a_theta1) / TF.a_theta3
K_theta_DC = pitch_kp * TF.a_theta3 / (TF.a_theta2 + pitch_kp * TF.a_theta3)

#----------altitude loop-------------
wn_altitude = wn_pitch / 30.0
zeta_altitude = 1.0
altitude_kp = 2.0 * zeta_altitude * wn_altitude / K_theta_DC / Va0
altitude_ki = wn_altitude**2 / K_theta_DC / Va0
altitude_zone = 10.0  # moving saturation limit around current altitude

#---------airspeed hold using throttle---------------
wn_airspeed_throttle = 1.5 #3.0
zeta_airspeed_throttle = 2.0  # 0.707
airspeed_throttle_kp = (2.0 * zeta_airspeed_throttle * wn_airspeed_throttle - TF.a_V1) / TF.a_V2
airspeed_throttle_ki = wn_airspeed_throttle**2 / TF.a_V2

#----------climb rate hold using pitch-------------
climbrate_kp = 1.0
climbrate_ki = 0.0
climbrate_kd = 1.0
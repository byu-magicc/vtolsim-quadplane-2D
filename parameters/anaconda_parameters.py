#This file contains the parameters for the anaconda aircraft


import numpy as np
from tools.rotations import euler_to_quaternion
from message_types.msg_delta import MsgDelta


######################################################################################
                #Initial Conditions
######################################################################################
#initial conditions for the QUADPLANE
pn0 = 0.0  # initial north position
pd0 = 0.0  # initial down position
pn_dot0 = 20.0  # initial velocity along inertial x-axis
pd_dot0 = 0.0  # initial velocity along inertial z-axis
theta0 = 0.0  # initial pitch angle
q0 = 0.0  # initial pitch rate
Va0 = np.sqrt(pn_dot0**2+pd_dot0**2)

######################################################################################
                #Physical Parameters
######################################################################################
mass = 11.0 #kg
Jy = 1.135
S_wing = 0.55 #surface are of the wing
c = 0.18994 #mean chord of the QUAD Wing
b = 2.8956 #wingspan
#other physical parameters
S_prop = 0.2027 #surface area of the propellor
rho = 1.2682 #air density
e = 0.9
AR = (b**2) / S_wing # aspect ratio
gravity = 9.81

#creates the gravity acceleration vector in the inertial frame
gravity_accel_inertial_2D = np.array([[0.0],[gravity]])

#creates the gravity acceleration vector in the inertial frame
gravity_accel_inertial_2D = np.array([[0.0],[gravity]])


#sets the physical positions of the props. That is, where their bases are located.
#in units of meters
ell_f = 0.5
ell_r = 0.5


#mixes individual thrusts to get the net thrust and torque
individualThrustMixer = np.array([[1, 1],
                                  [ell_f, -ell_r]])

#creates the mapping from total Thrust and Torque to front and rear thrusts
individualThrustUnmixer = np.linalg.inv(individualThrustMixer)



#######################################################################################
# Aerodynamic Coefficients
C_L_0 = 0.23#
C_D_0 = 0.043#
C_m_0 = 0.0135
C_L_alpha = 5.61#
C_D_alpha = 0.03#
C_m_alpha = -2.74
C_L_q = 7.95#
C_D_q = 0.0#
C_m_q = -38.21
C_L_delta_e = -0.13#
C_D_delta_e = 0.0135#
C_m_delta_e = -0.99
M = 50.0
alpha0 = np.deg2rad(15)
epsilon = 0.16
C_D_p = 0.0

######################################################################################
                #  Propeller parameters
######################################################################################
# # Prop parameters
D_prop = 20*(0.0254)     # prop diameter in m
# Motor parameters
KV_rpm_per_volt = 145.                            # Motor speed constant from datasheet in RPM/V
KV = (1. / KV_rpm_per_volt) * 60. / (2. * np.pi)  # Back-emf constant, KV in V-s/rad
KQ = KV                                           # Motor torque constant, KQ in N-m/A
R_motor = 0.042              # ohms
i0 = 1.5                     # no-load (zero-torque) current (A)
k_force = 65.0
k_moment = 5.0
# Inputs
ncells = 24 #12.
V_max = 3.7 * ncells  # max voltage for specified number of battery cells
#sets the maximum thrust
Tmax = 40
# Coeffiecients from prop_data fit
C_Q2 = -0.01664
C_Q1 = 0.004970
C_Q0 = 0.005230
C_T2 = -0.1079
C_T1 = -0.06044
C_T0 = 0.09357


#creates the prop direction 
propDirections = np.array([1.0, #front vertical prop
                           -1.0, #rear vertical prop
                           1.0]) #forward thrust prop


#defines the Maximum Thrust. TODO
#I calculated this by setting delta_t to 1.0 and then running through all the Va's
MaxThrust = 320.0


######################################################################################
                #  Trim parameters
######################################################################################

trim_elevator=-0.124778
trim_forwardThrottle=0.676752

#creates the trim message
trimDelta = MsgDelta(elevator=trim_elevator, 
                throttle_thrust=trim_forwardThrottle,
                throttle_front=0.0,
                throttle_rear=0.0)
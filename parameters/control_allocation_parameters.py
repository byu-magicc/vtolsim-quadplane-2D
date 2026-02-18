#This file implements the control allocation parameters for the functions


import numpy as np


#creates the scaling variable
scaling = 1.0

#sets the maximum expected Forces and Moments
Fx_bar = 100.0
Fz_bar = 100.0
My_bar = 50.0


#creates the K wrench control allocation mixing matrix
K_Wrench = scaling*np.diag([1/(Fx_bar**2), 1/(Fz_bar**2), 1/(My_bar**2)])


#creates the actuator bounds for the device
actuatorBounds = [(-1.0, 1.0), #elevator bounds
                  (0.0, 1.0), #front vertical throttle bounds
                  (0.0, 1.0), #rear vertical throttle bounds
                  (0.0, 1.0)] #forward thruster throttle bounds


# max iterations for nonlinear solver
max_iter = 50


#Pitch control parameters
pitchControl_riseTime = 2.0
pitchControl_zeta = 0.707

omega_n = np.pi / (2.0*pitchControl_riseTime*np.sqrt(1-pitchControl_zeta**2))

#sets the max tau
tau_max = 5.0

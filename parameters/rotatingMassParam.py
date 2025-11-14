#the parameters for the rotating mass param
import numpy as np

#sets the mass and rotational inertias
mass = 11.0 #kg
Jy = 1.135


#sets the initial conditions
theta0 = 0.0*np.pi/180
thetadot0 = 0.0*np.pi/180



#animation parameters
diameter = 1.0


#simulation parameters
t_start = 0.0
t_end = 50.0
Ts = 0.01
t_plot = 0.1


#saturation limits
tau_max = 2.0

#signal parameters
refAmplitude = 1.0
refFrequency = 0.2
#creates the file to store the parameters for pitch optimization
import numpy as np

#sets the maximum value of theta
theta_max = 30*np.pi/180
theta_min = -30*np.pi/180


#sets the max q
q_max = 5.0*np.pi/180


#sets alpha max (the maximum and minimum angle of attack)
alpha_max = 15*np.pi/180
alpha_min = -15*np.pi/180

#sets the maximum number of iterations
max_iter = 50


#creates the boundaries 
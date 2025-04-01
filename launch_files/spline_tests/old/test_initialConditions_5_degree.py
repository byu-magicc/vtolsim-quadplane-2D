#second iteration of the test for the initial conditions, using 5th degree splines and initial and final conditions
import numpy as np
from eVTOL_BSplines.path_generation_helpers.matrix_helpers import B_d_M_t_matrix
from bsplinegenerator.bsplines import BsplineEvaluation
from IPython.display import display
import matplotlib.pyplot as plt

#defines the initial acceleration
accel_init = -9.8

S = np.array([[0, 0, 0, 0, 0],
              [0, 0, accel_init, 0, 0]])


#sets the distance covered during the transition
transition_distance = 200.0
#sets the altitude
cruise_altitude = -10.0
#sets the cruise velocity
cruise_velocity = 25.0

#defines the End conditions for the takeoff
E = np.array([[transition_distance, cruise_velocity, 0, 0, 0],
              [cruise_altitude,     0,               0, 0, 0]])



#defines the M to be 9
M = 9

#defines the degree of the bspline to be 5
degree = 5


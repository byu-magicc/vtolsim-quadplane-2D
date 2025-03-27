#%%

#this file does the initial conditions for the aircraft and everything
import numpy as np
from eVTOL_BSplines.path_generation_helpers.matrix_helpers import B_d_M_t_matrix
from bsplinegenerator.bsplines import BsplineEvaluation
from IPython.display import display
import matplotlib.pyplot as plt


#this function works to obtain the initial conditions

#defines the initial acceleration
accel_init = -9.8


S = np.array([[0, 0, 0],
              [0, 0, accel_init]])


#sets the distance covered during the transition
transition_distance = 200.0
#sets the altitude
cruise_altitude = -10.0
#sets the cruise velocity
cruise_velocity = 25.0

#defines the End conditions for the takeoff
E = np.array([[transition_distance, cruise_velocity, 0.0],
              [cruise_altitude, 0, 0]])


#defines the M to be 9
M = 9

#defines the degree of the bspline to be 5
degree = 3

#gets the B_hat init matrix
B_init, B_hat_init = B_d_M_t_matrix(time=0.0,
                                    degree = degree,
                                    alpha=1,
                                    M=M)

#gets the B_had end matrix
B_end, B_hat_end = B_d_M_t_matrix(time=M,
                                  degree=degree,
                                  alpha=1,
                                  M=M)

#gets the B hat end inverse
B_hat_end_inv = np.linalg.inv(B_hat_end)

print("B hat init")
display(B_hat_init)
print("B hat end")
display(B_hat_end)
print("B hat end inverse: ")
display(B_hat_end_inv)

#gets the rank of the init matrix
print("rank of both: ", np.linalg.matrix_rank(B_hat_init))

#gets the start and end control points
C_init = S @ np.linalg.inv(B_hat_init)

C_end = E @ np.linalg.inv(B_hat_end)

x1 = C_end[0,:]
y1 = C_end[1,:]

print(C_init)
print(C_end)
plt.figure()
plt.scatter(x=C_init[0,:],y=-C_init[1,:], label='initial conditions')
plt.scatter(x=C_end[0,:], y=-C_end[1,:], label='test')
plt.legend()
plt.xlabel('x')
plt.ylabel('altitude')
plt.show()



# %%

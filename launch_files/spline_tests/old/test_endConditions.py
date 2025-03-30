#%%

#this file does the initial conditions for the aircraft and everything
import numpy as np
from eVTOL_BSplines.path_generation_helpers.matrix_helpers import B_d_M_t_matrix, uniform_knot_point_generator
from bsplinegenerator.bsplines import BsplineEvaluation
from bsplinegenerator.bspline_plotter import plot_bspline
from IPython.display import display
import matplotlib.pyplot as plt


#this function works to obtain the initial conditions

#defines the initial acceleration
accel_init = -9.8


S = np.array([[0, 25, 0],
              [-10.0, 0, 0]])


#sets the distance covered during the transition
transition_distance = 200.0
#sets the altitude
cruise_altitude = -10.0
#sets the cruise velocity
cruise_velocity = 25.0

#defines the End conditions for the takeoff
E = np.array([[100.0, 0, 0],
              [0, 0, 4.0]])


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

print("initial landing conditions")
print(C_init)
print("end landing conditions")
print(C_end)
plt.figure()
plt.scatter(x=C_init[0,:],y=-C_init[1,:], label='initial conditions')
plt.scatter(x=C_end[0,:], y=-C_end[1,:], label='end conditions')
plt.legend()
plt.xlabel('x')
plt.ylabel('altitude')
plt.show()



#creates the array for the intermediate points
intermediate_points = []
#sets the spacing
spacing = 80/7
start_x = 20
#iterates through and gets the new control points
for i in range(6):
    current_x = start_x + spacing*(i+1)
    #gets the current_y
    current_y = current_x/10 - 12
    intermediate_points.append(np.array([[current_x],
                                         [current_y]]))

#creates the the full control point array
intermediate_points = np.array(intermediate_points)[:,:,0].T

allControlPoints = np.concatenate((C_init, intermediate_points, C_end), axis=1)


#gets the knot points
knot_points = uniform_knot_point_generator(M=M, degree=degree, alpha=1, start_time=0.0)

#from the control points, get the bspline object
bspline = BsplineEvaluation(control_points=allControlPoints, order=degree, start_time=0.0, scale_factor=1.0, clamped=False)


#plots the spline
bspline.plot_spline(num_data_points_per_interval=10)

potato = 0


# %%

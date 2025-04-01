#%%
#we are going to structure this as follows: we will have the different points of interest.

import numpy as np
from eVTOL_BSplines.path_generation_helpers.matrix_helpers import B_hat_B_hat_inv, uniform_knot_point_generator
from eVTOL_BSplines.path_generation_helpers.matrix_helpers import initializeControlPoints
from eVTOL_BSplines.eVTOL_pathGeneration import eVTOL_PathGen
from eVTOL_BSplines.path_generation_helpers.conditions_helpers import conditions_d5
from bsplinegenerator.bsplines import BsplineEvaluation
from bsplinegenerator.bspline_plotter import plot_bspline
from IPython.display import display
import matplotlib.pyplot as plt



#creates the dimension
dimension = 2

#bspline degree will be 5th degree in this scenario
degree = 5

#gets the number of intervals of interest for the takeoff
M_takeoff = 10
#sets the number of intervals for the level flight
M_level = 15
#sets the number of intervals for the landing
M_landing = 10

#gets the total number of intervals of interest
M_total = M_takeoff + M_level + M_landing

#sets the M for the landing start
M_landing_start = M_takeoff + M_level


#initializes the eVTOL pathgen
pathGenerator = eVTOL_PathGen(degree=degree,
                              M=M_total,
                              start_time=0.0,
                              dimension=dimension,
                              alpha=1)


#sets the variables used for takeoff
takeoffDistance = 100.0
cruisingAltitude = -15.0
cruisingSpeed = 25.0


#sets the first few initial conditions
#sets the initial conditions for the takeoff leg
takeoff_pos_init = np.array([[0],[0]])
takeoff_vel_init = np.array([[0],[0]])
takeoff_accel_init = np.array([[0],[-5.0]])
takeoff_jerk_init = np.array([[0],[0]])
takeoff_snap_init = np.array([[0],[0]])

takeoff_init_conditions = conditions_d5(pos=takeoff_pos_init,
                                        vel=takeoff_vel_init,
                                        accel=takeoff_accel_init,
                                        jerk=takeoff_jerk_init,
                                        snap=takeoff_snap_init)

#calls the function to use the initial conditions to creates some control points
pathGenerator.setControlPoints_conditions(time=0.0,
                                          conditions=takeoff_init_conditions)




takeoff_final_conditions = conditions_d5(pos=np.array([[takeoffDistance],[cruisingAltitude]]),
                                         vel=np.array([[cruisingSpeed],[0.0]]),
                                         accel=np.array([[0],[0]]),
                                         jerk=np.array([[0],[0]]),
                                         snap=np.array([[0],[0]]))



#creates the control points at the final position OF the takeoff portion
pathGenerator.setControlPoints_conditions(time=M_takeoff,
                                          conditions=takeoff_final_conditions)



##########################################################################################

#sets the descent position
descent_start_x = 1000.0

#sets the descent distance
descent_distance = 300.0

descent_end_x = descent_start_x + descent_distance


#defines the landing initial conditions
landing_init_conditions = conditions_d5(pos=np.array([[descent_start_x],[cruisingAltitude]]),
                                        vel=np.array([[cruisingSpeed],[0]]),
                                        accel=np.array([[0],[0]]),
                                        jerk=np.array([[0],[0]]),
                                        snap=np.array([[0],[0]]))
pathGenerator.setControlPoints_conditions(time=M_landing_start,
                                          conditions=landing_init_conditions)

landing_final_conditions = conditions_d5(pos=np.array([[descent_end_x],[0.0]]),
                                         vel=np.array([[0.0],[0.1]]),
                                         accel=np.array([[0],[0]]),
                                         jerk=np.array([[0],[0]]),
                                         snap=np.array([[0],[0]]))

pathGenerator.setControlPoints_conditions(time=M_total,
                                          conditions=landing_final_conditions)





#calls the path generator automatic linear interpolation
pathGenerator.automaticLinearInterpolation()


#gets the control points from the path generator
controlPoints= pathGenerator.getControlPoints()
controlPoints_x = controlPoints[0,:]
controlPoints_z = controlPoints[1,:]




#with these control points, let us construct the bsplines
bspline = BsplineEvaluation(control_points=controlPoints,
                            order=degree,
                            start_time=0,
                            scale_factor=1,
                            clamped=False)

bspline.plot_spline(num_data_points_per_interval=20)



potato = 0




# %%

import eVTOL_BSplines as ebsp
from eVTOL_BSplines.uniform_path_generator import uniformPathGenerator
from eVTOL_BSplines.path_generation_helpers.waypoint_conditions import waypoint_conditions
import numpy as np

from bsplinegenerator.table_evaluation import cox_de_boor_table_basis_function

from bsplinegenerator.bsplines import BsplineEvaluation
from bsplinegenerator.bspline_plotter import plot_bspline
import matplotlib.pyplot as plt

#creates the control points

controlPoints = np.array([[-1,0,1,2,3,4,5,6],
                          [1,0,1,1,0,1,0,-1]])


#sets the number of data points per interval
numDataPoints = 50

#sets the order of the bspline
order = 2

bspline = BsplineEvaluation(control_points=controlPoints,
                            order=order,
                            start_time=0.0)



splineData, timeData = bspline.get_spline_data(num_data_points_per_interval=numDataPoints)

knotPointTimes = bspline.get_knot_points()
knotPointTimes = knotPointTimes.reshape((1,len(knotPointTimes)))
#evaluates the spline at the knot point times
knotPointEvaluations = []
for i in range(np.size(knotPointTimes)):
    evaluation = bspline.get_spline_at_time_t(time = knotPointTimes.item(i))
    knotPointEvaluations.append(evaluation)

knotPointEvaluations = np.array(knotPointEvaluations)[:,:,0].T


#using this bspline, we need to get the bspline basis functions at time t
#at time t = 0
b_d_M_0 = bspline.get_basis_functions_at_time_t(time = 0)
#at time end time
b_d_M_M = bspline.get_basis_functions_at_time_t(time = bspline._end_time)




plot_bspline(spline_data=splineData,
             knot_point_values=knotPointEvaluations,
             control_points=controlPoints)


potato = 0
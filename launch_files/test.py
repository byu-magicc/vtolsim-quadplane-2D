import eVTOL_BSplines as ebsp
from eVTOL_BSplines.uniform_path_generator import uniformPathGenerator
from eVTOL_BSplines.path_generation_helpers.waypoint_conditions import waypoint_conditions
import numpy as np

from bsplinegenerator.table_evaluation import cox_de_boor_table_basis_function


#sets the initial acceleration
init_accel = -1.0
#sets the end velocity
end_vel = 0.1

#gets the end x position using the uniform distribution
x_end = np.random.uniform(low=10.0, high=50.0)


#next we need to set the number of intermediate control points
numIntermediateControlPoints = 10

#creates the waypoint conditions, initial and end
initial_conditions = waypoint_conditions(position=np.array([[0.0],[0.0]]),
                                         velocity=np.array([[0.0],[0.0]]),
                                         acceleration=np.array([[0.0],[init_accel]]),
                                         jerk=np.array([[0.0],[0.0]]),
                                         snap=np.array([[0.0],[0.0]]))

end_conditions = waypoint_conditions(position=np.array([[x_end],[0.0]]),
                                     velocity=np.array([[0.0],[end_vel]]),
                                     acceleration=np.array([[0.0],[0.0]]),
                                     jerk=np.array([[0.0],[0.0]]),
                                     snap=np.array([[0.0],[0.0]]))


#creates the uniformPathGenerator
path_gen = uniformPathGenerator(dimension=2)

#calls the generate path function
path_gen.generate_path(initial_conditions=initial_conditions,
                       end_conditions=end_conditions,
                       max_altitude=10.0)





potato = 0
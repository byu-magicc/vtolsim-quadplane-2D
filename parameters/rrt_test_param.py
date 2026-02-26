from cvxpy import vec
import numpy as np
import parameters.anaconda_parameters as CONDA
from rrt_mavsim.tools.plane_projections_2 import map_3D_to_2D

startPosition_3D = np.array([[0.0],[0.0],[0.0]])
endPosition_3D = np.array([[2000.0],[0.0],[-1000.0]])

startPosition_2D = map_3D_to_2D(vec_3D=startPosition_3D,
                                plane=CONDA.plane_msg)

endPosition_2D = map_3D_to_2D(vec_3D=endPosition_3D,
                              plane=CONDA.plane_msg)

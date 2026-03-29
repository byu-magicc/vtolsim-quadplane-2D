from cvxpy import vec
import numpy as np
import parameters.anaconda_parameters as CONDA
from rrt_mavsim.tools.plane_projections_2 import map_3D_to_2D

startNorth = 0.0
startDown = -1000.0
startAltitude = -startDown

endNorth = 2000.0
endDown = 0.0
endAltitude = -1*endDown

startPosition_3D = np.array([[startNorth],[0.0],[startDown]])
endPosition_3D = np.array([[endNorth],[0.0],[endDown]])

startPosition_2D = map_3D_to_2D(vec_3D=startPosition_3D,
                                plane=CONDA.plane_msg)

endPosition_2D = map_3D_to_2D(vec_3D=endPosition_3D,
                              plane=CONDA.plane_msg)

x_limits = (-400, 400)
y_limits = (-200, endNorth+200)
z_limits = (-200, startAltitude+200)

x_range = x_limits[1] - x_limits[0]
y_range = y_limits[1] - y_limits[0]
z_range = z_limits[1] - z_limits[0]

#gets the max
max_range = max(x_range, y_range, z_range)
#creates the list  for the aspect ratio
aspect_ratio = [x_range/max_range, y_range/max_range, z_range/max_range]

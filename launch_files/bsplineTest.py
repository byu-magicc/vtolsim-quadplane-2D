#implements the test for the b-splines
import os, sys
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))
import numpy as np
from rrt_mavsim.message_types.msg_world_map import MsgWorldMap, PlanarVTOLParams, MapTypes
from rrt_mavsim.viewers.view_manager import ViewManager
from rrt_mavsim.planners.rrt_sfc_bspline import RRT_SFC_BSpline
from rrt_mavsim.planners.bspline_generator import BSplineGenerator
import rrt_mavsim.parameters.planner_parameters as PLAN
import rrt_mavsim.parameters.flightCorridor_parameters as FLIGHT_PLAN
import rrt_mavsim.parameters.planarVTOL_map_parameters as VTOL_PARAM
from viewers.view_manager import ViewManager
from rrt_mavsim.tools.smoothingTools import flight_corridors_smooth_path
from rrt_mavsim.tools.waypointsTools import getNumCntPts_list, getInitialFinalControlPoints
from rrt_mavsim.tools.plane_projections import map_2D_to_3D
from bsplinegenerator.bsplines import BsplineEvaluation

#current file path
currentFilePath = Path(__file__).resolve()
mainDirectoryFilePath = currentFilePath.parents[1]
#sets the subfolder
lookupDirectory = mainDirectoryFilePath / 'lookupTables'

mapFileName = 'worldMap.npz'
waypointsSmoothFileName = 'waypointsSmooth.npz'
waypointsNotSmoothFileName = 'waypointsNotSmooth.npz'

mapDirectory = lookupDirectory / mapFileName
waypointsSmoothDirectory = lookupDirectory / waypointsSmoothFileName
waypointsNotSmoothDirectory = lookupDirectory / waypointsNotSmoothFileName

#loads up those lookup tables. Unpacks it from the python pickling process
worldMapData = np.load(mapDirectory, allow_pickle=True)
waypointsSmoothData = np.load(waypointsSmoothDirectory, allow_pickle=True)
waypointsNotSmoothData = np.load(waypointsNotSmoothDirectory, allow_pickle=True)


worldMap = worldMapData['model'].item()
waypoints_smooth = waypointsSmoothData['model'].item()
waypoints_not_smooth = waypointsNotSmoothData['model'].item()

mapOrigin_2D = np.array([[0.0],[0.0]])
mapOrigin_3D = np.array([[0.0],[0.0],[0.0]])
n_hat = np.array([[0.0],[1.0],[0.0]])

#now that we have the world map and the waypoints, both smooth and unsmooth, let us plot them out

viewer = ViewManager(animation=True,
                     world_map=worldMap)

viewer.drawWaypoints(waypoints=waypoints_not_smooth,
                     n_hat=n_hat,
                     p0=mapOrigin_3D,
                     color='b')

#draws
viewer.drawWaypoints(waypoints=waypoints_smooth,
                     n_hat=n_hat,
                     p0=mapOrigin_3D,
                     color='r')


purple = np.array([[170, 0, 255],
                   [170, 0, 255]])/255



#calls the function to create the B-Spline Path
bspline_gen = BSplineGenerator(numDimensions=worldMap.numDimensions_algorithm,
                               degree=FLIGHT_PLAN.degree,
                               M=FLIGHT_PLAN.M)

#gets the path of control points through the SFCs based on the SFCs
outputControlPoints_2D = bspline_gen.generatePath(waypoints=waypoints_smooth,
                                               numPointsPerUnit=FLIGHT_PLAN.numPoints_perUnit)


#gets the same ouptut control points in 3D
outputControlPoints_3D = map_2D_to_3D(pos_2D=outputControlPoints_2D,
                                      n_hat=n_hat,
                                      p0=mapOrigin_3D)

#gets the spline object
bspline_object = BsplineEvaluation(control_points=outputControlPoints_3D,
                                   order=FLIGHT_PLAN.degree,
                                   start_time=0.0)

#draws the sampled points
bspline_sampledPoints, bspline_timeData = bspline_object.get_spline_data(num_data_points_per_interval=100)

viewer.drawTrajectory(controlPoints=outputControlPoints_3D,
                      sampledPoints_spline=bspline_sampledPoints,
                      lineColor=purple,
                      lineWidth=2.0,
                      pointWidth=4.0)

#TODO make colors reference package


potato = 0
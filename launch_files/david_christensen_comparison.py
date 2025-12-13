#implements the autopilot test for an rrt autopilot test
import os, sys
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))
import numpy as np
from copy import deepcopy
import parameters.simulation_parameters as SIM
import parameters.anaconda_parameters as ANP
import parameters.plane_parameters as PLANE

from models.quadplaneDynamics import QuadplaneDynamics
from viewers.view_manager import ViewManager


#controllers import section
from controllers.highLevelControl import highLevelControl
from controllers.LowLevelControl import LowLevelControl
from controllers.feedforwardControl import feedForwardControl

#imports the message types from 
from message_types.msg_delta import MsgDelta
from message_types.msg_state import MsgState
from message_types.msg_sensors import MsgSensors
from rrt_mavsim.message_types.msg_plane import MsgPlane
from message_types.msg_trajectory import MsgTrajectory

from rrt_mavsim.message_types.msg_world_map import MsgWorldMap, PlanarVTOLParams, MapTypes
from rrt_mavsim.planners.rrt_sfc_bspline import RRT_SFC_BSpline
import rrt_mavsim.parameters.planner_parameters as PLAN
import rrt_mavsim.parameters.flightCorridor_parameters as FLIGHT_PLAN
import rrt_mavsim.parameters.planarVTOL_map_parameters as VTOL_PARAM
from rrt_mavsim.tools.smoothingTools import flight_corridors_smooth_path
from rrt_mavsim.tools.waypointsTools import getNumCntPts_list
from rrt_mavsim.planners.bspline_generator import BSplineGenerator
from rrt_mavsim.tools.plane_projections import *
from rrt_mavsim.parameters.colors import *
from rrt_mavsim.tools.plane_projections import map_3D_to_2D_planeMsg

from rrt_mavsim.tools.plane_projections import *
import rrt_mavsim.parameters.planner_parameters as PLAN
import rrt_mavsim.parameters.flightCorridor_parameters as FLIGHT_PLAN
from rrt_mavsim.planners.david_christensen_wrapper import PathOptimizer

from eVTOL_BSplines.submodules.path_generator.path_generation.waypoint_data import Waypoint, WaypointData


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
numDimensions = 2

rho = np.array([[1.0],[1.0],[1.0]])

#initializes the MsgPlane
msg_plane = MsgPlane(n_hat=n_hat,
                     origin_3D=mapOrigin_3D)

#gets the plane basis
Q = getPlaneBasis(n_hat=n_hat)


#creates the planar vtol params
params = PlanarVTOLParams(mapOrigin_2D=mapOrigin_2D,
                          mapOrigin_3D=mapOrigin_3D,
                          n_hat=n_hat)

startPosition_3D = params.startPosition
endPosition_3D = params.endPosition

#gets the 2D start and end positions
startPosition_2D = map_3D_to_2D_planeMsg(vec_3D=startPosition_3D,
                                         plane_msg=msg_plane)

endPosition_2D = map_3D_to_2D_planeMsg(vec_3D=endPosition_3D,
                                       plane_msg=msg_plane)


#instantiates the quadplane
quadplane = QuadplaneDynamics(ts=SIM.ts_simulation,
                              plane_msg=PLANE.plane_msg,
                              vel_3D_0=np.array([[25.0],[0.0],[0.0]]))


viewers = ViewManager(animation=True, 
                      data=True,
                      video=False,
                      world_map=worldMap,
                      msg_plane=msg_plane)


viewers.drawWaypoints(waypoints=waypoints_not_smooth,
                     n_hat=n_hat,
                     p0=mapOrigin_3D,
                     color='b')

#draws the smooth set of waypoints
viewers.drawWaypoints(waypoints=waypoints_smooth,
                     n_hat=n_hat,
                     p0=mapOrigin_3D,
                     color='r')



#now, with teh waypoints, we generate the control points using the above waypoints

bsplineGen = RRT_SFC_BSpline(numDimensions=numDimensions,
                             degree=FLIGHT_PLAN.degree,
                             M=FLIGHT_PLAN.M,
                             Va=PLAN.Va0,
                             rho=FLIGHT_PLAN.rho,
                             step_length=FLIGHT_PLAN.segmentLength,
                             numDesiredInitPaths=FLIGHT_PLAN.numInitialPaths,
                             plane=msg_plane)

#gets the output control points
rrt_sfc_outputControlPoints_2D = bsplineGen.generateControlPoints(waypoints=waypoints_smooth,
                                                                  numPointsPerUnit=FLIGHT_PLAN.numPoints_perUnit)

#creates the optimizer to optimize the path beyond what is being done currently
path_optimizer = PathOptimizer(dimension=numDimensions,
                               degree=3)


#creates the start and end waypoints in 2D frame
startVel = np.array([[0.0],[1.0]])
startAccel = np.array([[0.0],[0.0]])

startWaypoint = Waypoint(location=startPosition_2D,
                         velocity=startVel,
                         acceleration=startAccel)

endVel = np.array([[25.0],[0.0]])
endAccel = np.array([[0.0],[0.0]])

endWaypoint = Waypoint(location=endPosition_2D,
                       velocity=endVel,
                       acceleration=endAccel)

waypoint_data = WaypointData(start_waypoint=startWaypoint,
                             end_waypoint=endWaypoint)


path_optimizer.generate_path(waypoint_data=waypoint_data,
                             world_map=worldMap,
                             plane=msg_plane,
                             controlPoints_init=rrt_sfc_outputControlPoints_2D)


potato = 0
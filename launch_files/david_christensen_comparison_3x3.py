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

from rrt_mavsim.message_types.msg_world_map import MsgWorldMap, PlanarVTOLSimplifiedParams, MapTypes
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

from rrt_mavsim.parameters.colors import *


field_width = 1500.0

numDimensions = 2

startPosition_2D = np.array([[0.0],[0.0]])
endPosition_2D = np.array([[field_width],[field_width]])

startVelocity_2D = np.array([[0.0],[1.0]])
endVelocity_2D = np.array([[25.0],[0.0]])

startAccel_2D = np.array([[0.0],[0.0]])
endAccel_2D = np.array([[0.0],[0.0]])

#plane params
mapOrigin_3D = np.array([[0.0],[0.0],[0.0]])
n_hat = np.array([[0.0],[1.0],[0.0]])
#initializes the MsgPlane
msg_plane = MsgPlane(n_hat=n_hat,
                     origin_3D=mapOrigin_3D)

startPosition_3D = map_2D_to_3D_planeMsg(vec_2D=startPosition_2D,
                                         plane_msg=msg_plane)
endPosition_3D = map_2D_to_3D_planeMsg(vec_2D=endPosition_2D,
                                       plane_msg=msg_plane)


#creates the params for the vtol
params = PlanarVTOLSimplifiedParams(plane=msg_plane,
                                    fieldLength=field_width)

#creates the world map
world_map = MsgWorldMap(obstacleFieldType=MapTypes.PLANAR_VTOL_SIMPLIFIED,
                        numDimensions_algorithm=2,
                        planarVTOLSimplified_Params=params)



#creates the planner and then plans the path on the world map
planner = RRT_SFC_BSpline(numDimensions=2,
                          degree=FLIGHT_PLAN.degree,
                          M=FLIGHT_PLAN.M,
                          rho=FLIGHT_PLAN.rho,
                          Va=PLAN.Va0,
                          step_length=FLIGHT_PLAN.segmentLength,
                          numDesiredInitPaths=FLIGHT_PLAN.numInitialPaths,
                          plane=msg_plane)



waypoints_not_smooth = planner.generateSFCPaths(startPosition_3D=startPosition_3D,
                                                endPosition_3D=endPosition_3D,
                                                worldMap=world_map,
                                                segmentLength=FLIGHT_PLAN.segmentLength)

#generates the control points for the not smooth waypoints
controlPoints_2D = planner.generateControlPoints(waypoints=waypoints_not_smooth,
                                              numPointsPerUnit=FLIGHT_PLAN.numPoints_perUnit)

#gets them in 3D
controlPoints_3D = map_2D_to_3D_planeMsg(vec_2D=controlPoints_2D,
                                         plane_msg=msg_plane)

#gets the bspline
bspline = BsplineEvaluation(control_points=controlPoints_3D,
                             order=FLIGHT_PLAN.degree,
                             start_time=0.0)

sampledPoints, _ = bspline.get_spline_data(num_data_points_per_interval=100)


viewer = ViewManager(data=True,
                     animation=True,
                     draw_trajectory=True,
                     world_map=world_map,
                     msg_plane=msg_plane)

viewer.drawWaypoints(waypoints=waypoints_not_smooth,
                     n_hat=n_hat,
                     p0=mapOrigin_3D)

viewer.drawTrajectory(controlPoints=controlPoints_3D,
                      sampledPoints_spline=sampledPoints,
                      lineColor=purple,
                      lineWidth=5.0,
                      pointWidth=10.0)


#instantiates the path optimizer
path_optimizer = PathOptimizer(dimension=numDimensions)

startWaypoint = Waypoint(location=startPosition_2D,
                         velocity=startVelocity_2D,
                         acceleration=startAccel_2D)

endWaypoint = Waypoint(location=endPosition_2D,
                       velocity=endVelocity_2D,
                       acceleration=endAccel_2D)

#generates the waypoint data
waypoint_data = WaypointData(start_waypoint=startWaypoint,
                             end_waypoint=endWaypoint)

path_optimizer.generate_path(waypoint_data=waypoint_data,
                             world_map=world_map,
                             plane=msg_plane,
                             controlPoints_init=sampledPoints)

potato = 0
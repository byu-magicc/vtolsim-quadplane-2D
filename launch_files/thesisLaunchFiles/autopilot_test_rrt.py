# generates the bspline via the evtolbsplines minimum snap path gen based on start and end conditions
# and then uses the autopilot to command to those.
import os, sys
from pathlib import Path

sys.path.insert(0, os.fspath(Path(__file__).parents[2]))
import numpy as np
from copy import deepcopy
import matplotlib.pyplot as plt

import parameters.simulation_parameters as SIM
import parameters.anaconda_parameters as CONDA
import parameters.safe_flight_corridor_parameters as SFC
import parameters.rrt_test_param as RRT
import pandas as pd

from models.quadplaneDynamics import QuadplaneDynamics
from viewers.view_manager import ViewManager

from controllers.highLevelControl import highLevelControl
from controllers.LowLevelControl import LowLevelControl

from message_types.msg_delta import MsgDelta
from message_types.msg_state import MsgState
from message_types.msg_sensors import MsgSensors
from rrt_mavsim.message_types.msg_plane import MsgPlane
from message_types.msg_trajectory import MsgTrajectory
from rrt_mavsim.parameters.colors import *
from rrt_mavsim.tools.plane_projections_2 import map_2D_to_3D, map_3D_to_2D
from rrt_mavsim.viewers.plot_map_path import PlotMapPath

#import section for the rrt stuff
from rrt_mavsim.message_types.msg_world_map import MsgWorldMap
from rrt_mavsim.planners.rrt_sfc_bspline import RRT_SFC_BSpline
from rrt_mavsim.message_types.msg_world_map import MsgWorldMap, PlanarVTOLSimplifiedParams, MapTypes

from eVTOL_BSplines.path_generation_helpers.staticFlightPath import staticFlightPath
from bsplinegenerator.bsplines import BsplineEvaluation
from planners.trajectoryGenerator import trajectoryGenerator

#section to create the planar VTOL map
#creates the path generator
path_gen = RRT_SFC_BSpline(numDimensions=SFC.numDimensions,
                           degree=SFC.degree,
                           M=SFC.M,
                           Va=CONDA.Va0,
                           rho=SFC.rho_bspline,
                           step_length=SFC.segmentLength,
                           numDesiredInitPaths=SFC.numInitPaths,
                           plane=CONDA.plane_msg,
                           chiMax=SFC.Chi_max)

planar_params = PlanarVTOLSimplifiedParams(plane=CONDA.plane_msg,)

world_map = MsgWorldMap(obstacleFieldType=MapTypes.PLANAR_VTOL_SIMPLIFIED,
                        numDimensions_algorithm=SFC.numDimensions,
                        planarVTOLSimplified_Params=planar_params)


viewers = ViewManager(
    world_map=world_map,
    animation=True, 
    data=True, 
    pathPlot=True, 
    video=False, 
    video_name="takeoff", 
    msg_plane=CONDA.plane_msg
)


path_gen.generateSFCPaths(startPosition_3D=RRT.startPosition_3D,
                          endPosition_3D=RRT.endPosition_3D,
                          worldMap=world_map,
                          segmentLength=SFC.segmentLength)

#gets the smooth and not smooth waypoints
waypoints_not_smooth = path_gen.getWaypointsNotSmooth()

waypoints_smooth = path_gen.getWaypointsSmooth()

#draws the waypoints, starting with the waypoints_not_smooth
viewers.drawWaypoints(waypoints=waypoints_not_smooth, color='blue')
viewers.drawWaypoints(waypoints=waypoints_smooth, color='purple')

#creates the control points
controlPoints_notSmooth_2D = path_gen.generateControlPoints(waypoints=waypoints_not_smooth,
                                                         numPointsPerUnit=0.01)

#evaluates the B-Spline for the not smooth control points
bspline_notSmooth = BsplineEvaluation(control_points=controlPoints_notSmooth_2D,
                                      order=3,
                                      start_time=0.0)
#samples the not smooth bspline
splineSampledPoints_notSmooth_2D, timeData = bspline_notSmooth.get_spline_data(num_data_points_per_interval=100)

#maps the 2D points to 3D
controlPoints_notSmooth_3D = map_2D_to_3D(vec_2D=controlPoints_notSmooth_2D,
                                          plane=CONDA.plane_msg)
splineSampledPoints_notSmooth_3D = map_2D_to_3D(vec_2D=splineSampledPoints_notSmooth_2D,
                                                plane=CONDA.plane_msg)

#draws the not smooth trajecotry
viewers.drawTrajectory(controlPoints=controlPoints_notSmooth_3D,
                       sampledPoints_spline=splineSampledPoints_notSmooth_3D,
                       lineColor=np.array([[1.,1.,0.,1.],
                                           [1.,1.,0.,1.]]),
                       lineWidth=2.0,
                       pointWidth=5.0)

#creates the control points
controlPoints_smooth_2D = path_gen.generateControlPoints(waypoints=waypoints_smooth,
                                                         numPointsPerUnit=0.01)

#evaluates the B-Spline for the not smooth control points
bspline_smooth = BsplineEvaluation(control_points=controlPoints_smooth_2D,
                                      order=3,
                                      start_time=0.0)
#samples the not smooth bspline
splineSampledPoints_smooth_2D, timeData = bspline_smooth.get_spline_data(num_data_points_per_interval=100)

#maps the 2D points to 3D
controlPoints_smooth_3D = map_2D_to_3D(vec_2D=controlPoints_smooth_2D,
                                          plane=CONDA.plane_msg)
splineSampledPoints_smooth_3D = map_2D_to_3D(vec_2D=splineSampledPoints_smooth_2D,
                                                plane=CONDA.plane_msg)

#draws the not smooth trajecotry
viewers.drawTrajectory(controlPoints=controlPoints_smooth_3D,
                       sampledPoints_spline=splineSampledPoints_smooth_3D,
                       lineColor=np.array([[1.,156.0/255.0,0.,1.],
                                           [1.,156.0/255.0,0.,1.]]),
                       lineWidth=2.0,
                       pointWidth=5.0)


# instantiates the quadplane
quadplane = QuadplaneDynamics(
    ts=SIM.ts_simulation,
    plane_msg=CONDA.plane_msg,
    pos_3D_inertial_init=RRT.startPosition_3D,
    vel_3D_inertial_init=startVelocity_3D,
)




testPoint = 0

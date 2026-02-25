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

planar_params = PlanarVTOLSimplifiedParams(plane=CONDA.plane_msg)
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


testPoint = 0

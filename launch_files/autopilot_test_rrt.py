#implements the autopilot test for an rrt autopilot test
import os, sys
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))
import numpy as np
from copy import deepcopy
import parameters.simulation_parameters as SIM
import parameters.anaconda_parameters as ANP
from models.quadplane_dynamics import QuadplaneDynamics
from viewers.view_manager import ViewManager
from controllers.autopilot_quadrotors import Autopilot

#controllers import section
from controllers.highLevelControl import highLevelControl
from controllers.LowLevelControl import LowLevelControl
from controllers.feedforwardControl import feedForwardControl

#imports the message types from 
from message_types.msg_delta import MsgDelta
from message_types.msg_state import MsgState
from message_types.msg_sensors import MsgSensors
from message_types.msg_plane import MsgPlane
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

#initializes the MsgPlane
msg_plane = MsgPlane(n_hat=n_hat,
                     origin_3D=mapOrigin_3D)


#creates the planar vtol params
params = PlanarVTOLParams(mapOrigin_2D=mapOrigin_2D,
                          mapOrigin_3D=mapOrigin_3D,
                          n_hat=n_hat)

startPosition = params.startPosition
endPosition = params.endPosition



#instantiates the quadplane
quadplane = QuadplaneDynamics(ts=SIM.ts_simulation)
viewers = ViewManager(animation=True, 
                      data=True,
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


#calls the function to create the B-Spline Path
bspline_gen = BSplineGenerator(numDimensions=worldMap.numDimensions_algorithm,
                               degree=FLIGHT_PLAN.degree,
                               M=FLIGHT_PLAN.M)

#gets the path of control points through the SFCs based on the SFCs
outputControlPoints_2D = bspline_gen.generatePath(waypoints=waypoints_smooth,
                                               numPointsPerUnit=FLIGHT_PLAN.numPoints_perUnit)


#gets the bspline sampled points in 2D, and then the velocity and acceleration points as well
bspline_object_2D = BsplineEvaluation(control_points=outputControlPoints_2D,
                                      order=FLIGHT_PLAN.degree,
                                      start_time=0.0)

#section to get the data for the position of the bspline's samples
bspline_sampledPositions_2D, bspline_timeData_2D = bspline_object_2D.get_spline_data(num_data_points_per_interval=100)
bspline_sampledVelocity_2D, _ = bspline_object_2D.get_spline_derivative_data(num_data_points_per_interval=100,
                                                                             rth_derivative=1)
bspline_sampledAcceleration_2d, _ = bspline_object_2D.get_spline_derivative_data(num_data_points_per_interval=100,
                                                                                 rth_derivative=2)


#gets the same ouptut control points in 3D
outputControlPoints_3D = map_2D_to_3D(pos_2D=outputControlPoints_2D,
                                      n_hat=n_hat,
                                      p0=mapOrigin_3D)


#gets the spline object
bspline_object_3D = BsplineEvaluation(control_points=outputControlPoints_3D,
                                   order=FLIGHT_PLAN.degree,
                                   start_time=0.0)

#draws the sampled points
bspline_sampledPoints_3D, bspline_timeData_3D = bspline_object_3D.get_spline_data(num_data_points_per_interval=100)



#gets the time spacing between time data samples
timeSpacing = bspline_timeData_3D.item(1) - bspline_timeData_3D.item(0)


viewers.drawTrajectory(controlPoints=outputControlPoints_3D,
                      sampledPoints_spline=bspline_sampledPoints_3D,
                      lineColor=purple,
                      lineWidth=2.0,
                      pointWidth=4.0)


#creates the controller
high_level_controller = highLevelControl(state=quadplane.true_state)

#creates the low level controller
low_level_controller = LowLevelControl()

#creates the instance for the reference state class
trajectory_ref = MsgTrajectory()


#instantiates the autopilot
autopilot = None


#creates the wind
wind = np.array([[0.0],[0.0],[0.0],[0.0]])


sim_time = SIM.start_time
end_time = SIM.end_time


#iterates through until we get to the end time
while sim_time < end_time:


    #gets the current time index
    currentTimeIndex = int(sim_time / timeSpacing)

    #gets the current desired positions, velocities, and accelerations
    pos_desired = bspline_sampledPositions_2D[:,currentTimeIndex].reshape(-1,1)
    vel_desired = bspline_sampledVelocity_2D[:,currentTimeIndex].reshape(-1,1)
    accel_desired = bspline_sampledAcceleration_2d[:,currentTimeIndex].reshape(-1,1)

    #updates the trajectory reference (does not do theta yet.)
    trajectory_ref.update(pos=pos_desired,
                          vel=vel_desired,
                          accel=accel_desired)

    #gets the forces and moments desired in the body frame of the aircraft
    F_des_b, M_des_b = high_level_controller.update(trajectory_ref=trajectory_ref,
                                 state=quadplane.true_state)

    #calls the low level controller
    low_level_controller.update(state=quadplane.true_state,
                                F_des_b=F_des_b,
                                M_des_b=M_des_b)


    #creates the dummy delta 
    delta_temp = MsgDelta(elevator=0.0,
                          throttle_front=0.0,
                          throttle_rear=0.0,
                          throttle_thrust=0.0)
    
    
    #updates the quadplane dynamic simulation based on the delta input
    quadplane.update(delta=delta_temp,
                     wind=wind)
    


    viewers.update(sim_time=sim_time,
                   true_state=quadplane.true_state,
                   estimated_state=quadplane.true_state,
                   commanded_state=quadplane.true_state,
                   delta=delta_temp,
                   measurements=MsgSensors())

    sim_time += SIM.ts_simulation


potato = 0
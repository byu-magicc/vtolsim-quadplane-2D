# generates the bspline via the evtolbsplines minimum snap path gen based on start and end conditions
# and then uses the autopilot to command to those.
import os, sys
from pathlib import Path

from bsplinegenerator.bspline_to_minvo import convert_to_minvo_control_points

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
                                                         numPointsPerUnit=0.05)

#evaluates the B-Spline for the not smooth control points
bspline_smooth = BsplineEvaluation(control_points=controlPoints_smooth_2D,
                                      order=3,
                                      start_time=0.0)
#samples the not smooth bspline
splineSampledPoints_smooth_2D, timeData = bspline_smooth.get_spline_data(num_data_points_per_interval=100)

splineVelocityPoints_smooth_2D, _ = bspline_smooth.get_spline_derivative_data(
    num_data_points_per_interval=100, rth_derivative=1
)

splineAccelPoints_smooth_2D, _ = bspline_smooth.get_spline_derivative_data(
    num_data_points_per_interval=100, rth_derivative=2
)

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

staticGen = staticFlightPath()

startControlPoints_3D = controlPoints_smooth_3D[:,:3]

startConditions = staticGen.getLocalizedConditions(controlPoints=startControlPoints_3D,
                                                   d=3,
                                                   M=10)

startPositionTemp_3D = startConditions[:,0:1]
startVelocityTemp_3D = startConditions[:,1:2]


testPoint = 0

# instantiates the quadplane
quadplane = QuadplaneDynamics(
    ts=SIM.ts_simulation,
    plane_msg=CONDA.plane_msg,
    pos_3D_inertial_init=startPositionTemp_3D,
    vel_3D_inertial_init=startVelocityTemp_3D,
)


# creates the controller
high_level_controller = highLevelControl(state=quadplane.true_state,
                                         plane=CONDA.plane_msg)

# creates the low level controller
low_level_controller = LowLevelControl()

# creates the instance for the reference state class
trajectory_ref = MsgTrajectory()

# creates the wind
wind = np.array([[0.0], [0.0], [0.0], [0.0]])

# gets the time spacing between time data samples
timeSpacing = timeData.item(1) - timeData.item(0)



#section to create the lists to store the data for later analysis
desiredPosition_list = []
desiredVelocity_list = []
desiredAcceleration_list = []

actualPosition_list = []
actualVelocity_list = []

timeArray_list = []
time_list = []
deltasList = []

#creates the gamma_ref and the gamma current lists
theta_list = []
gamma_list = []
gamma_ref_list = []
alpha_list = []


#bools to help us only stop at two points for plotting
firstStopUnvisited = True
secondStopUnvisited = True

num_sampled_positions = splineSampledPoints_smooth_2D.shape[1]

counter = 0

sim_time = SIM.start_time
end_time = 220.0

# iterates through until we get to the end time
while sim_time < end_time and counter < num_sampled_positions:
    # gets the current time index
    currentTimeIndex = int(sim_time / timeSpacing)

    # gets the current desired positions, velocities, and accelerations
    pos_desired = splineSampledPoints_smooth_2D[:, currentTimeIndex].reshape(-1, 1)
    vel_desired = splineVelocityPoints_smooth_2D[:, currentTimeIndex].reshape(-1, 1)
    accel_desired = splineAccelPoints_smooth_2D[:, currentTimeIndex].reshape(-1, 1)

    desiredPosition_list.append(pos_desired.T)
    desiredVelocity_list.append(vel_desired.T)
    desiredAcceleration_list.append(accel_desired.T)

    # updates the trajectory reference (does not do theta yet.)
    trajectory_ref.update(pos=pos_desired, vel=vel_desired, accel=accel_desired)

    # gets the forces and moments desired in the body frame of the aircraft
    F_des_b, M_des_b = high_level_controller.update(
        trajectory_ref=trajectory_ref, state=quadplane.true_state
    )

    integrator = high_level_controller.getIntegrator()

    # calls the low level controller
    delta = low_level_controller.update(
        state=quadplane.true_state, F_des_body=F_des_b, M_des_body=M_des_b
    )

    deltasList.append((delta.to_array()).T)

    pos_actual = quadplane.true_state.pos_2D
    vel_actual = quadplane.true_state.vel_2D
    
    actualPosition_list.append(pos_actual.T)
    actualVelocity_list.append(vel_actual.T)


    # updates the quadplane dynamic simulation based on the delta input
    quadplane.update(delta=delta, wind=wind)

    viewers.update(
        sim_time=sim_time,
        true_state=quadplane.true_state,
        estimated_state=quadplane.true_state,
        commanded_state=quadplane.true_state,
        delta=delta,
        measurements=MsgSensors(),
        integrator=integrator,
        trajectory=trajectory_ref,
    )

    time_list.append(sim_time)
    timeArray_list.append(np.array([[sim_time]]))

    theta = quadplane.true_state.theta
    theta_list.append(theta)
    gamma = quadplane.true_state.gamma
    gamma_ref = trajectory_ref.gamma_ref
    gamma_list.append(gamma)
    gamma_ref_list.append(gamma_ref)
    alpha_list.append(quadplane.true_state.alpha)


    testPoint = 0

    counter += 1
    sim_time += SIM.ts_simulation


#saves the various components
timeArray = np.concatenate((timeArray_list), axis=0)
positionsActual_array = np.concatenate((actualPosition_list), axis=0)
velocitiesActual_array = np.concatenate((actualVelocity_list), axis=0)

positionsRef_array = np.concatenate((desiredPosition_list), axis=0)
velocitiesRef_array = np.concatenate((desiredVelocity_list), axis=0)
delta_array = np.concatenate((deltasList), axis=0)

thetaArray = np.array(theta_list).reshape((len(theta_list),1))
gammaArray = np.array(gamma_list).reshape((len(gamma_list),1))
gammaRefArray = np.array(gamma_ref_list).reshape((len(gamma_ref_list),1))
alphaArray = np.array(alpha_list).reshape((len(alpha_list),1))
anglesArray = np.concatenate((thetaArray, gammaArray, gammaRefArray, alphaArray), axis=1)


df1 = pd.DataFrame(timeArray)
df1.to_csv('multipleInclinesCSV/times.csv', index=False, header=False)

df2 = pd.DataFrame(positionsActual_array)
df2.to_csv('multipleInclinesCSV/positionsActual.csv', index=False, header=False)

df3 = pd.DataFrame(velocitiesActual_array)
df3.to_csv('multipleInclinesCSV/velocitiesActual.csv', index=False, header=False)

df4 = pd.DataFrame(positionsRef_array)
df4.to_csv('multipleInclinesCSV/positionsRef.csv', index=False, header=False)

df5 = pd.DataFrame(velocitiesRef_array)
df5.to_csv('multipleInclinesCSV/velocitiesRef.csv', index=False, header=False)

df6 = pd.DataFrame(delta_array)
df6.to_csv('multipleInclinesCSV/deltas.csv', index=False, header=False)

df7 = pd.DataFrame(anglesArray)
df7.to_csv('multipleInclinesCSV/angles.csv', index=False, header=False)

testPoint = 0




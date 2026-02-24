#generates a simple inclined line for testing the autopilot
import os, sys
from pathlib import Path

sys.path.insert(0, os.fspath(Path(__file__).parents[2]))
import numpy as np
from copy import deepcopy
import matplotlib.pyplot as plt

import parameters.simulation_parameters as SIM
import parameters.anaconda_parameters as CONDA
import pandas as pd

from models.quadplaneDynamics import QuadplaneDynamics
from viewers.view_manager import ViewManager

# controllers import section
from controllers.highLevelControl import highLevelControl
from controllers.LowLevelControl import LowLevelControl
from controllers.feedforwardControl import feedForwardControl

# imports the message types from
from message_types.msg_delta import MsgDelta
from message_types.msg_state import MsgState
from message_types.msg_sensors import MsgSensors
from rrt_mavsim.message_types.msg_plane import MsgPlane
from message_types.msg_trajectory import MsgTrajectory
from rrt_mavsim.parameters.colors import *
from rrt_mavsim.tools.plane_projections_2 import map_2D_to_3D, map_3D_to_2D

from eVTOL_BSplines.path_generation_helpers.staticFlightPath import staticFlightPath
from bsplinegenerator.bsplines import BsplineEvaluation
from planners.trajectoryGenerator import trajectoryGenerator


northSeperation = 400.0

#creates the gamma list
gamma_deg_list = [0.0,
                  10.0,
                  20.0,
                  30.0]

velocity = 25.0

gamma_list = [np.radians(gamma_deg_temp) for gamma_deg_temp in gamma_deg_list]

positions_list = []
velocities_list = []

previous_end_down = 0.0

for i, gamma in enumerate(gamma_list):
    current_start_north = i*northSeperation
    current_end_north = (i+1)*northSeperation

    delta_down = -northSeperation*np.tan(gamma)

    current_start_down = previous_end_down
    current_end_down = current_start_down + delta_down

    previous_end_down = current_end_down

    #case we append the start
    if i == 0:
        positions_list.append(np.array([[current_start_north],[current_start_down]]))

    positions_list.append(np.array([[current_end_north],[current_end_down]]))

    velocities_list.append(velocity)
    


startPosition_2D = positions_list[0]
#gets the start velocity
startVelocity_2D = positions_list[1] - positions_list[0]
startVelocity_2D = startVelocity_2D / np.linalg.norm(startVelocity_2D)
startVelocity_2D = velocity*startVelocity_2D
startAccel_2D = np.array([[0.0],[0.0]])
#creates the 2D start conditions
startConditions_2D = [startPosition_2D, startVelocity_2D, startAccel_2D]
startPosition_3D = map_2D_to_3D(vec_2D=startPosition_2D, plane=CONDA.plane_msg)
startVelocity_3D = map_2D_to_3D(vec_2D=startVelocity_2D, plane=CONDA.plane_msg)


start_gamma = gamma_list[0]

rho = np.array([1.0,1.0,1.0])

traj_gen = trajectoryGenerator(plane=CONDA.plane_msg,
                               rho=rho)

controlPoints = traj_gen.generateLinearConnectedTrajectory(secondaryPointsList=positions_list,
                                                           velocityList=velocities_list,
                                                           start_conditions=startConditions_2D)

bspline_object = BsplineEvaluation(
    control_points=controlPoints, order=3, start_time=0.0, scale_factor=1
)

# section to get the data for the position of the bspline's samples
bspline_sampledPositions_2D, bspline_timeData_2D = bspline_object.get_spline_data(
    num_data_points_per_interval=100
)

#gets the number of sampled positions
num_sampled_positions = bspline_sampledPositions_2D.shape[1]

#TODO remove this section
#gets the numerical derivative for my own sanity
delta_time = bspline_timeData_2D.item(1) - bspline_timeData_2D.item(0)

bspline_sampledVelocity_2D, _ = bspline_object.get_spline_derivative_data(
    num_data_points_per_interval=100, rth_derivative=1
)

bspline_sampledAcceleration_2d, _ = bspline_object.get_spline_derivative_data(
    num_data_points_per_interval=100, rth_derivative=2
)

# gets the same ouptut control points in 3D
outputControlPoints_3D = map_2D_to_3D(vec_2D=controlPoints,
                                      plane=CONDA.plane_msg)

# gets the spline object
bspline_object_3D = BsplineEvaluation(
    control_points=outputControlPoints_3D, order=3, start_time=0.0
)

# draws the sampled points
bspline_sampledPoints_3D, bspline_timeData_3D = bspline_object_3D.get_spline_data(
    num_data_points_per_interval=100
)

viewers = ViewManager(
    animation=True, data=True, pathPlot=True, video=False, video_name="takeoff", msg_plane=CONDA.plane_msg
)

viewers.drawTrajectory(
    controlPoints=outputControlPoints_3D,
    sampledPoints_spline=bspline_sampledPoints_3D,
    lineColor=purple,
    lineWidth=2.0,
    pointWidth=4.0,
)

# instantiates the quadplane
quadplane = QuadplaneDynamics(
    ts=SIM.ts_simulation,
    plane_msg=CONDA.plane_msg,
    pos_3D_inertial_init=startPosition_3D,
    vel_3D_inertial_init=startVelocity_3D,
    theta0=start_gamma,
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
timeSpacing = bspline_timeData_3D.item(1) - bspline_timeData_3D.item(0)


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


counter = 0

sim_time = SIM.start_time
end_time = 220.0

# iterates through until we get to the end time
while sim_time < end_time and counter < num_sampled_positions:
    # gets the current time index
    currentTimeIndex = int(sim_time / timeSpacing)

    # gets the current desired positions, velocities, and accelerations
    pos_desired = bspline_sampledPositions_2D[:, currentTimeIndex].reshape(-1, 1)
    vel_desired = bspline_sampledVelocity_2D[:, currentTimeIndex].reshape(-1, 1)
    accel_desired = bspline_sampledAcceleration_2d[:, currentTimeIndex].reshape(-1, 1)

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

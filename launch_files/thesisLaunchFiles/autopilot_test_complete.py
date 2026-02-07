
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
import parameters.plane_parameters as PLANE
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
from rrt_mavsim.tools.plane_projections import *
from rrt_mavsim.parameters.colors import *

from eVTOL_BSplines.path_generation_helpers.staticFlightPath import staticFlightPath

from bsplinegenerator.bsplines import BsplineEvaluation

from planners.trajectoryGenerator import trajectoryGenerator

polynomialDegree = 1/3

straightLength = 2000.0
transitionLength = 500.0

north_startTakeoff = 0.0
north_endTakeoff = north_startTakeoff + transitionLength
north_startLanding = north_endTakeoff + straightLength
north_endLanding = north_startLanding + transitionLength

cruisingVelocity = 25.0



startConditions_takeoff = [np.array([[north_startTakeoff],[0.0],[0.0]]),
                   np.array([[0.0],[0.0],[-1.0]]),
                   np.array([[0.0],[0.0],[0.0]])]

startPosition_3D = startConditions_takeoff[0]
startVelocity_3D = startConditions_takeoff[1]

endConditions_takeoff = [np.array([[north_endTakeoff],[0.0],[-100.0]]),
                   np.array([[cruisingVelocity],[0.0],[0.0]]),
                   np.array([[0.0],[0.0],[0.0]])]
#'''

#'''
#Landing
startConditions_landing = [np.array([[north_startLanding],[0.0],[-100.0]]),
                   np.array([[cruisingVelocity],[0.0],[0.0]]),
                   np.array([[0.0],[0.0],[0.0]])]


endConditions_landing = [np.array([[north_endLanding],[0.0],[0.0]]),
                   np.array([[0.0],[0.0],[1.0]]),
                   np.array([[0.0],[0.0],[0.0]])]
#'''

mapOrigin_2D = np.array([[0.0],[0.0]])
mapOrigin_3D = np.array([[0.0],[0.0],[0.0]])
n_hat = np.array([[0.0],[1.0],[0.0]])

plane_msg = MsgPlane(n_hat=n_hat,
                     origin_3D=mapOrigin_3D)

rho = np.array([1.0,1.0,1.0])

gen = trajectoryGenerator(plane=plane_msg,
                          rho=rho)

controlPoints = gen.generateCompleteTrajectory(startConditions_takeoff=startConditions_takeoff,
                               endConditions_takeoff=endConditions_takeoff,
                               startConditions_landing=startConditions_landing,
                               endConditions_landing=endConditions_landing,
                               polynomialDegree=polynomialDegree)



bspline_object = BsplineEvaluation(
    control_points=controlPoints, order=3, start_time=0.0, scale_factor=2
)

# section to get the data for the position of the bspline's samples
bspline_sampledPositions_2D, bspline_timeData_2D = bspline_object.get_spline_data(
    num_data_points_per_interval=100
)
bspline_sampledVelocity_2D, _ = bspline_object.get_spline_derivative_data(
    num_data_points_per_interval=100, rth_derivative=1
)
bspline_sampledAcceleration_2d, _ = bspline_object.get_spline_derivative_data(
    num_data_points_per_interval=100, rth_derivative=2
)


# gets the same ouptut control points in 3D
outputControlPoints_3D = map_2D_to_3D(
    vec_2D=controlPoints, n_hat=n_hat, p0=mapOrigin_3D
)


# gets the spline object
bspline_object_3D = BsplineEvaluation(
    control_points=outputControlPoints_3D, order=3, start_time=0.0
)

# draws the sampled points
bspline_sampledPoints_3D, bspline_timeData_3D = bspline_object_3D.get_spline_data(
    num_data_points_per_interval=100
)


viewers = ViewManager(
    animation=True, data=True, video=False, video_name="takeoff", msg_plane=plane_msg
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
    plane_msg=PLANE.plane_msg,
    pos_3D_inertial_init=startPosition_3D,
    vel_3D_inertial_init=startVelocity_3D,
)

# creates the controller
high_level_controller = highLevelControl(state=quadplane.true_state)

# creates the low level controller
low_level_controller = LowLevelControl()

# creates the instance for the reference state class
trajectory_ref = MsgTrajectory()

# creates the wind
wind = np.array([[0.0], [0.0], [0.0], [0.0]])


# gets the time spacing between time data samples
timeSpacing = bspline_timeData_3D.item(1) - bspline_timeData_3D.item(0)

sim_time = SIM.start_time
end_time = 220.0

#section to create the lists to store the data for later analysis
desiredPosition_list = []
desiredVelocity_list = []
desiredAcceleration_list = []

actualPosition_list = []
actualVelocity_list = []

time_list = []

deltasList = []


# iterates through until we get to the end time
while sim_time < end_time:
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
        state=quadplane.true_state, F_des_b=F_des_b, M_des_b=M_des_b
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

    time_list.append(np.array([[sim_time]]))
    sim_time += SIM.ts_simulation


timeArray = np.concatenate(time_list, axis=0)
df_m1 = pd.DataFrame(timeArray)
df_m1.to_csv('launch_files/thesisLaunchFiles/completePathCSV/times.csv', index=False, header=False)

deltasArray = np.concatenate(deltasList, axis=0)
df_m2 = pd.DataFrame(deltasArray)
df_m2.to_csv('launch_files/thesisLaunchFiles/completePathCSV/deltas.csv', index=False, header=False)


testPoint = 0

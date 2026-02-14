# generates the bspline via the evtolbsplines minimum snap path gen based on start and end conditions
# and then uses the autopilot to command to those.
import os, sys
from pathlib import Path
import pandas as pd

sys.path.insert(0, os.fspath(Path(__file__).parents[2]))
import numpy as np
from copy import deepcopy

import parameters.simulation_parameters as SIM
import parameters.anaconda_parameters as CONDA


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
from rrt_mavsim.tools.plane_projections_2 import map_2D_to_3D, map_3D_to_2D
from rrt_mavsim.parameters.colors import *


from eVTOL_BSplines.path_generation_helpers.staticFlightPath import staticFlightPath

from bsplinegenerator.bsplines import BsplineEvaluation

from planners.takeoffGenerator import flightPathGenerator, pathTypes

startVelocity = 25.0
endVelocity = 1.0
startAccel = 0.0
endAccel = 0.0

# creates the conditions for the trajectory to be generated
startPos_3D = np.array([[0.0], [0.0], [-100.0]])
startVel_3D = np.array([[startVelocity], [0.0], [0.0]])
startAccel_3D = np.array([[startAccel], [0.0], [0.0]])

startConditions_3D = [startPos_3D, startVel_3D, startAccel_3D]

endPos_3D = np.array([[500.0], [0.0], [0.0]])
endVel_3D = np.array([[0.0], [0.0], [endVelocity]])
endAccel_3D = np.array([[0.0], [0.0], [endAccel]])

endConditions_3D = [endPos_3D, endVel_3D, endAccel_3D]


startConditions_2D = []
endConditions_2D = []

for condition_3D in startConditions_3D:
    condition_2D = map_3D_to_2D(vec_3D=condition_3D,
                                plane=CONDA.plane_msg)
    startConditions_2D.append(condition_2D)

for condition_3D in endConditions_3D:
    condition_2D = map_3D_to_2D(vec_3D=condition_3D,
                                plane=CONDA.plane_msg)
    endConditions_2D.append(condition_2D)


rho = np.array([1.0, 1.0, 1.0])


takeoffGen = flightPathGenerator(plane=CONDA.plane_msg, rho=rho, numDimensions=2, d=3, M=10)

controlPoints = takeoffGen.generatePath(
    pathType=pathTypes.PARABOLA_LANDING,
    startConditions_3D=startConditions_3D,
    endConditions_3D=endConditions_3D,
)


testPoint = 0

#"""
bspline_object = BsplineEvaluation(
    control_points=controlPoints, order=3, start_time=0.0, scale_factor=2.0
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
    vec_2D=controlPoints, plane=CONDA.plane_msg
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
    animation=True, data=True, video=False, video_name="takeoff", msg_plane=CONDA.plane_msg
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
    pos_3D_inertial_init=startPos_3D,
    vel_3D_inertial_init=startVel_3D,
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

time_list = []

sim_time = SIM.start_time
end_time = 60.0


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
        state=quadplane.true_state, F_des_body=F_des_b, M_des_body=M_des_b
    )


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
df_m1.to_csv('launch_files/thesisLaunchFiles/landingCSV/times.csv', index=False, header=False)

actualPositions = np.concatenate((actualPosition_list), axis = 0)
df0 = pd.DataFrame(actualPositions)
df0.to_csv('launch_files/thesisLaunchFiles/landingCSV/ActualPositions.csv', index=False, header=False)

actualVelocities = np.concatenate((actualVelocity_list), axis = 0)
df1 = pd.DataFrame(actualVelocities)
df1.to_csv('launch_files/thesisLaunchFiles/landingCSV/actualVelocities.csv', index=False, header=False)

desiredVelocities = np.concatenate((desiredVelocity_list), axis = 0)
df2 = pd.DataFrame(desiredVelocities)
df2.to_csv('launch_files/thesisLaunchFiles/landingCSV/desiredVelocities.csv', index=False, header=False)

desiredPositions = np.concatenate((desiredPosition_list), axis = 0)
df3 = pd.DataFrame(desiredPositions)
df3.to_csv('launch_files/thesisLaunchFiles/landingCSV/desiredPositions.csv', index=False, header=False)

desiredAccelerations = np.concatenate((desiredAcceleration_list), axis = 0)
df4 = pd.DataFrame(desiredAccelerations)
df4.to_csv('launch_files/thesisLaunchFiles/landingCSV/desiredAccelerations.csv', index=False, header=False)


potato = 0
#"""

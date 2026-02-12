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
from message_types.msg_integrator import MsgIntegrator
from rrt_mavsim.message_types.msg_plane import MsgPlane
from message_types.msg_trajectory import MsgTrajectory
from rrt_mavsim.tools.plane_projections import *
from rrt_mavsim.parameters.colors import *

from eVTOL_BSplines.path_generation_helpers.staticFlightPath import staticFlightPath

from bsplinegenerator.bsplines import BsplineEvaluation

from planners.takeoffGenerator import flightPathGenerator, pathTypes


# creates the start position in 3D
startPos_3D = np.array([[0.0], [0.0], [0.0]])
startVel_3D = np.array([[0.0], [0.0], [0.0]])
startAccel_3D = np.array([[0.0], [0.0], [0.0]])

startConditions_3D = [startPos_3D, startVel_3D, startAccel_3D]

mapOrigin_2D = np.array([[0.0], [0.0]])
mapOrigin_3D = np.array([[0.0], [0.0], [0.0]])
n_hat = np.array([[0.0], [1.0], [0.0]])

plane_msg = MsgPlane(n_hat=n_hat, origin_3D=mapOrigin_3D)

rho = np.array([1.0, 1.0, 1.0])

viewers = ViewManager(
    animation=True, data=True, video=False, video_name="takeoff", msg_plane=plane_msg
)

# instantiates the quadplane
quadplane = QuadplaneDynamics(
    ts=SIM.ts_simulation,
    plane_msg=PLANE.plane_msg,
    pos_3D_0=startPos_3D,
    vel_3D_0=startVel_3D,
)

# creates the controller
high_level_controller = highLevelControl(state=quadplane.true_state)

# creates the low level controller
low_level_controller = LowLevelControl()

# creates the instance for the reference state class
trajectory_ref = MsgTrajectory()

# creates the wind
wind = np.array([[0.0], [0.0], [0.0], [0.0]])



sim_time = SIM.start_time
end_time = 60.0

#section to create the lists to store the data for later analysis
desiredPosition_list = []
desiredVelocity_list = []
desiredAcceleration_list = []

actualPosition_list = []
actualVelocity_list = []

time_list = []


# iterates through until we get to the end time
while sim_time < end_time:


    delta = MsgDelta()

    # updates the quadplane dynamic simulation based on the delta input
    quadplane.update(delta=delta, wind=wind)

    viewers.update(
        sim_time=sim_time,
        true_state=quadplane.true_state,
        estimated_state=quadplane.true_state,
        commanded_state=quadplane.true_state,
        delta=delta,
        measurements=MsgSensors(),
    )

    sim_time += SIM.ts_simulation

potato = 0

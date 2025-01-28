#This file implements the simulation file, which runs a trimmed flight model for the aircraft.



import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))


import numpy as np
import parameters.anaconda_parameters as QP
from parameters.anaconda_parameters import trimDelta

from models.quadplane_dynamics import QuadplaneDynamics
import parameters.simulation_parameters as SIM
from viewers.view_manager import ViewManager
from message_types.msg_sensors import MsgSensors


import pandas as pd

from copy import copy

#creates the quadplane dynamics
quad = QuadplaneDynamics(ts=SIM.ts_control)
#creates the viewer
viewers = ViewManager(data=True, animation=True)


sim_time = SIM.start_time
end_time = SIM.end_time



#creates the simulation time
while sim_time < end_time:


    #sets the current wind
    current_wind = np.array([[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]])
    #updates the quad
    quad.update(delta=trimDelta, wind=current_wind)

    viewers.update(
        sim_time,
        true_state=quad.true_state,
        commanded_state=quad.true_state,
        delta=trimDelta,
        measurements=MsgSensors(),
        estimated_state=quad.true_state
    )


    #increments the time
    sim_time += SIM.ts_simulation
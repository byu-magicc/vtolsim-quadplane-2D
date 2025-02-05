#This file implements the controller for the vertical autopilot for the aircraft

import os, sys
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))
import numpy as np
import pyqtgraph as pg
import parameters.simulation_parameters as SIM
import parameters.anaconda_parameters as QP
from models.quadplane_dynamics import QuadplaneDynamics
from tools.trim import compute_trim, compute_ss_model, print_ss_model
from controllers.autopilot_quadrotors import Autopilot
from viewers.view_manager import ViewManager
from message_types.msg_delta import MsgDelta
from message_types.msg_autopilot import MsgAutopilot_Quadrotor
from message_types.msg_sensors import MsgSensors
import time

import pandas as pd


#instantiates the dynamics for the quadplane
quadplane = QuadplaneDynamics(SIM.ts_simulation)
viewers = ViewManager(animation=True, data=True)

#instantiates the autopilot
autopilot = Autopilot(ts_control=SIM.ts_control)

#creates the autopilot commands
autopilot_commands = MsgAutopilot_Quadrotor()


sim_time = SIM.start_time
end_time = SIM.end_time


while sim_time < end_time:

    current_wind = np.zeros((4, 1))

    delta, commanded_state = autopilot.update(command=autopilot_commands,
                                              state=quadplane.true_state)
    

    #updates the quadplane dynamics
    quadplane.update(delta=delta, wind=current_wind)

    #updates the viewers thing
    viewers.update(sim_time=sim_time,
                   true_state=quadplane.true_state,
                   estimated_state=quadplane.true_state,
                   commanded_state=quadplane.true_state,
                   delta=delta,
                   measurements=MsgSensors())
    
    #increments the sim time
    sim_time += SIM.ts_simulation

    time.sleep(SIM.sleep_time)



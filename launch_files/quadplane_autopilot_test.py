#This file implements a test for the quadplane autopilot 2d case, 
# which is just airspeed and altitude commands.

import os, sys
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))
import numpy as np
np.errstate(overflow="ignore")
from copy import deepcopy
import parameters.simulation_parameters as SIM
import parameters.anaconda_parameters as QP
from models.quadplane_dynamics import QuadplaneDynamics
from tools.trim import compute_trim, compute_ss_model, print_ss_model
from controllers.autopilot_fixedWing import Autopilot
from viewers.view_manager import ViewManager
from message_types.msg_delta import MsgDelta
from message_types.msg_autopilot import MsgAutopilot
from message_types.msg_sensors import MsgSensors
import time



#initializes the quadplane and the viewer
quadplane = QuadplaneDynamics(SIM.ts_simulation)
viewers = ViewManager(animation=True, data=True)

#instantiates the autopilot
autopilot = Autopilot(ts_control=SIM.ts_control)


#creates the command for the autopilot
autopilot_commands = MsgAutopilot()

#sets the autopilot commands
autopilot_commands.airspeed_command = 25
autopilot_commands.altitude_command = 110




#sets the start and end time
sim_time = SIM.start_time
end_time = SIM.end_time


#iterates through all the possible times
while sim_time < end_time:

    current_wind = np.zeros((4, 1))

    #gets the delta from the commands
    delta = autopilot.update(cmd=autopilot_commands,
                             state=quadplane.true_state)
    
    #updates the quadplane based on those inputs
    quadplane.update(delta=delta, wind=current_wind)

    #updates the viewers
    viewers.update(sim_time=sim_time,
                   true_state=quadplane.true_state,
                   estimated_state=quadplane.true_state,
                   commanded_state=quadplane.true_state,
                   delta=delta,
                   measurements=MsgSensors())
    

    #increments the simulation time
    sim_time += SIM.ts_simulation

    time.sleep(SIM.sleep_time)
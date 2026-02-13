#This file implements a test for the quadplane autopilot 2d case, 
# which is just airspeed and altitude commands.

import os, sys
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))
import numpy as np
np.errstate(overflow="ignore")
from copy import deepcopy
import parameters.simulation_parameters as SIM
import parameters.anaconda_parameters as CONDA
from models.old.quadplane_dynamics_old_thrust import QuadplaneDynamics
from tools.trim import compute_trim, compute_ss_model, print_ss_model
from controllers.autopilot_fixedWing import Autopilot
from viewers.view_manager import ViewManager
from message_types.msg_delta import MsgDelta
from message_types.msg_autopilot import MsgAutopilot_FixedWing
from message_types.msg_sensors import MsgSensors
import time


import pandas as pd


#initializes the quadplane and the viewer
quadplane = QuadplaneDynamics(SIM.ts_simulation)
viewers = ViewManager(animation=True, data=True)

#instantiates the autopilot
autopilot = Autopilot(ts_control=SIM.ts_control)


#creates the command for the autopilot
autopilot_commands = MsgAutopilot_FixedWing()

#sets the autopilot commands
autopilot_commands.airspeed_command = 25
autopilot_commands.altitude_command = 0.0



#list to store the forces and moments
wrenches = []


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

    #gets the wrenches
    currentWrench = quadplane._forces_moments(delta=delta)
    wrenches.append(currentWrench)

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



#creates the output path
outputPath = os.path.abspath("launch_files/outputFiles")


wrenches = (np.array(wrenches)[:,:,0]).T
dataFrame = pd.DataFrame(wrenches)
dataFrame.to_csv(outputPath + "/controlWrenches.csv", header=False, index=False)

potato = 0
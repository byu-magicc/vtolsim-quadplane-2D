#This file implements the controller for the low level controller, 
#and implements a test file case where we see if we can do 
#the low level force and torque controls.

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
from message_types.msg_autopilot import MsgAutopilot_FixedWing
from message_types.msg_sensors import MsgSensors
import time


import pandas as pd


#initializes the quadplane and the viewer
quadplane = QuadplaneDynamics(SIM.ts_simulation)
viewers = ViewManager(animation=True, data=True)
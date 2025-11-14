import os, sys
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))

import numpy as np
from controllers.LowLevelControl import LowLevelControl
from models.quadplane_dynamics import QuadplaneDynamics
import parameters.simulation_parameters as SIM
import parameters.anaconda_parameters as QP
import parameters.control_allocation_parameters as CAP
from viewers.view_manager import ViewManager
from message_types.msg_delta import MsgDelta
from message_types.msg_autopilot import MsgAutopilot_FixedWing
from controllers.wrenchCalculation import wrenchCalculator

from tools.rotations import *
import scipy.optimize as spo
import matplotlib.pyplot as plt



quadplane = QuadplaneDynamics(ts=SIM.ts_simulation)

lowlevel_controller = LowLevelControl(Ts=SIM.ts_simulation,
                                      pitchControl_riseTime=CAP.pitchControl_riseTime,
                                      pitchControl_zeta=CAP.pitchControl_zeta)



potato = 0
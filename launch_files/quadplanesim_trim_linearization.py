"""
quadplanesim_trim_linearization
    - compute trim for level flight and associated state space equations
    - verify that the eVTOL remains in trim
    - Update history:
        1/16/2025 - RWB
"""

import os, sys
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))
import numpy as np
np.errstate(overflow="ignore")
from copy import deepcopy
import parameters.simulation_parameters as SIM
from models.quadplane_dynamics import QuadplaneDynamics
from tools.trim import compute_trim, compute_ss_model, print_ss_model
from viewers.view_manager import ViewManager
#from message_types.msg_delta import MsgDelta

# initialize elements of the architecture
quadplane = QuadplaneDynamics(SIM.ts_simulation)
viewers = ViewManager(animation=True, data=True)
#viewers = ViewManager(animation=True)

# # compute trim and state space models at zeros airspeed (hover)
# Va = 0.
# trim_state, trim_delta = compute_trim(quadplane, Va)
# quadplane._state = trim_state  # set the initial state of the vtol to the trim state
# delta = trim_delta  # set input to constant constant trim input
# A, B = compute_ss_model(quadplane, trim_state, trim_delta)
# print_ss_model('ss_model_Va_0.py', A, B, Va, trim_state, trim_delta)

# compute trim and state space models at zeros airspeed (hover)
Va = 20.
trim_state, trim_delta = compute_trim(quadplane, Va)
quadplane._state = trim_state  # set the initial state of the vtol to the trim state
delta = trim_delta  # set input to constant constant trim input
A, B = compute_ss_model(quadplane, trim_state, trim_delta)
print_ss_model('ss_model_Va_20.py', A, B, Va, trim_state, trim_delta)

# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    #-------update physical system-------------
    current_wind = np.zeros((4, 1))
    #delta = MsgDelta()
    quadplane.update(delta, current_wind)  

    #-------update viewers-------------
    viewers.update(
        sim_time,
        quadplane.true_state,  # true states
        quadplane.true_state,  # estimated states
        quadplane.true_state,  # commanded states
        delta,  # inputs to aircraft
        None,  # measurements
    )    
    #-------increment time-------------
    sim_time += SIM.ts_simulation





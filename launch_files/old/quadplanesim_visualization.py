"""
quadplanesim-visualization
        1/16/2025 - RWB
"""
import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))
# import view manager
from viewers.view_manager import ViewManager
from tools.old.rotations import euler_to_rotation
import parameters.simulation_parameters as SIM
from message_types.msg_state import MsgState

# initialize messages
state = MsgState()  # instantiate state message

# initialize viewers and video
viewers = ViewManager(animation=True)

# initialize the simulation time
sim_time = SIM.start_time
psi = 0
theta = 0
phi = 0
end_time = 4000
# main simulation loop
while sim_time < end_time:
    # -------vary states to check viewer-------------
    if sim_time < end_time/8:
        state.pos_2D[0] += 10*SIM.ts_simulation
    elif sim_time < 2*end_time/8:
        state.pos_2D[1] += 10*SIM.ts_simulation
    elif sim_time < 3*end_time/8:
        state.pos_2D[2] -= 10*SIM.ts_simulation
    elif sim_time < 4*end_time/8:
        psi += 0.01*SIM.ts_simulation
    elif sim_time < 5*end_time/8:
        theta += 0.01*SIM.ts_simulation
    elif sim_time < 6*end_time/8:
        phi += 0.01*SIM.ts_simulation
    elif sim_time < 7*end_time/8:
        state.motor_angle[0,0] += 0.01*SIM.ts_simulation  # right motor
    else:
        state.motor_angle[1,0] += 0.01*SIM.ts_simulation  # left motor
    state.R = euler_to_rotation(phi, theta, psi)

    #-------update viewers-------------
    viewers.update(
        sim_time,
        state,  # true states
        state,  # estimated states
        state,  # commanded states
        None,  # inputs to aircraft
        None,  # measurements
    )    

    # -------increment time-------------
    sim_time += SIM.ts_simulation





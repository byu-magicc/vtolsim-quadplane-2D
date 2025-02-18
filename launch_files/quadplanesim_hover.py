"""
quadplanesim_hover
    - hover control using LQR and trimmed over model
    - Update history:
        1/23/2025 - RWB
"""

import os, sys
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))
import numpy as np
import parameters.simulation_parameters as SIM
from tools.signal_generator import SignalGenerator
from models.quadplane_dynamics import QuadplaneDynamics
from message_types.msg_trajectory import MsgTrajectory
from controllers.trajectory_tracker import TrajectoryTracker
from controllers.control_allocator import ControlAllocator
from viewers.view_manager import ViewManager

# initialize elements of the architecture
quadplane = QuadplaneDynamics(SIM.ts_simulation, pn_dot0=0.0, pd_dot0=0.0)
tracker = TrajectoryTracker(SIM.ts_simulation)
allocator = ControlAllocator(SIM.ts_simulation)
trajectory = MsgTrajectory()
viewers = ViewManager(animation=True, data=True)

# trajectory definition (step response)
pn_command = SignalGenerator(dc_offset=0, amplitude=0, 
                             start_time=5.0, frequency = 0.0025)
h_command = SignalGenerator(dc_offset=10, amplitude=0, 
                            start_time=0.0, frequency = 0.0025)

# initialize the simulation time
sim_time = SIM.start_time
end_time = 100.

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < end_time:
    #-------estimator-------------
    estimated_state = quadplane.true_state  
    #-------trajectory generator-------------
    trajectory.pos = np.array([
            [pn_command.square(sim_time)],
            [-h_command.square(sim_time)],
        ])
    # trajectory velocity, acceleration, pitch, and pitch rate all default to zero
    # -------controller-------------
    estimated_state = quadplane.true_state  # uses true states in the control
    commanded_wrench, commanded_state = tracker.update(trajectory, estimated_state)
    delta = allocator.update(commanded_wrench, estimated_state)
    #-------update physical system-------------
    current_wind = np.zeros((4, 1))
    quadplane.update(delta, current_wind)  
    #-------update viewers-------------
    viewers.update(
        sim_time,
        quadplane.true_state,  # true states
        estimated_state,  # estimated states
        commanded_state,  # commanded states
        delta,  # inputs to aircraft
        None,  # measurements
    )    
    #-------increment time-------------
    sim_time += SIM.ts_simulation





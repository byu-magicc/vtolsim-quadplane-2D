#implements the autopilot test for an rrt autopilot test
import os, sys
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))
import numpy as np
from copy import deepcopy
import parameters.simulation_parameters as SIM
import parameters.anaconda_parameters as ANP
from models.quadplane_dynamics import QuadplaneDynamics
from viewers.view_manager import ViewManager
from controllers.autopilot_quadrotors import Autopilot


#instantiates the quadplane
quadplane = QuadplaneDynamics(ts=SIM.ts_simulation)
viewers = ViewManager(animation=True, data=True)


#instantiates the autopilot
autopilot = None
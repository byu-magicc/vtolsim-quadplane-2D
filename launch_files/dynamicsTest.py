#implements a dynamics test
import os, sys
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))

import numpy as np

from models.quadplaneDynamics import QuadplaneDynamics

from rrt_mavsim.message_types.msg_plane import MsgPlane

import parameters.plane_parameters as PLANE
import parameters.simulation_parameters as SIM
import parameters.anaconda_parameters as CONDA




#creates the plane message




quadplane = QuadplaneDynamics(plane_msg=PLANE.plane_msg,
                              ts=SIM.ts_simulation,
                              pn0_3D=100.,
                              pd0_3D=50.,
                              pn_dot0_3D=25.,
                              pd_dot0_3D=-10.,
                              theta0=np.radians(30),
                              q0=0.)
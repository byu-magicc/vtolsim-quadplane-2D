import os, sys
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))

import numpy as np
from models.quadplaneDynamics import QuadplaneDynamics
from rrt_mavsim.message_types.msg_plane import MsgPlane
from rrt_mavsim.tools.plane_projections_2 import *


n_hat = np.array([[0.0],[-1.0],[0.0]])
origin_3D = np.array([[0.0],[0.0],[0.0]])
plane = MsgPlane(n_hat=n_hat, origin_3D=origin_3D)


quad = QuadplaneDynamics(plane_msg=plane,
                         pos_3D_inertial_init=np.array([[0.0],[0.0],[0.0]]),
                         vel_3D_inertial_init=np.array([[1.0],[0.0],[0.0]]),
                         theta0=np.radians(-30),
                         q0=0.0)


testPoint = 0

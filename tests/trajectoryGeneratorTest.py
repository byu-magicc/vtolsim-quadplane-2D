import numpy as np
import os, sys
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))

from planners.trajectoryGenerator import trajectoryGenerator, pathTypes

from rrt_mavsim.message_types.msg_plane import MsgPlane
import matplotlib.pyplot as plt

startConditions = [np.array([[0.0],[0.0],[0.0]]),
                   np.array([[0.0],[0.0],[-1.0]]),
                   np.array([[0.0],[0.0],[-1.0]])]


endConditions = [np.array([[500.0],[0.0],[-100.0]]),
                   np.array([[25.0],[0.0],[0.0]]),
                   np.array([[0.0],[0.0],[0.0]])]

mapOrigin_2D = np.array([[0.0],[0.0]])
mapOrigin_3D = np.array([[0.0],[0.0],[0.0]])
n_hat = np.array([[0.0],[1.0],[0.0]])

plane_msg = MsgPlane(n_hat=n_hat,
                     origin_3D=mapOrigin_3D)

rho = np.array([1.0,1.0,1.0])

gen = trajectoryGenerator(plane=plane_msg,
                          rho=rho)





gen.generateParabola_takeoff(startConditions_3D=startConditions,
                             endConditions_3D=endConditions)



testPoint = 0

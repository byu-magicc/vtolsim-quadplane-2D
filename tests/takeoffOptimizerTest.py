import os, sys
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))

from planners.takeoffOptimizer import takeoffOptimizer
import numpy as np
from rrt_mavsim.message_types.msg_plane import MsgPlane

import matplotlib.pyplot as plt



mapOrigin_2D = np.array([[0.0],[0.0]])
mapOrigin_3D = np.array([[0.0],[0.0],[0.0]])
n_hat = np.array([[0.0],[1.0],[0.0]])

plane_msg = MsgPlane(n_hat=n_hat,
                     origin_3D=mapOrigin_3D)

rho = np.array([1.0,1.0,1.0])

d = 3
M = 10

endVelocity = 25.0
endAltitude = 100.0

#creates the start conditions in the 3D
startConditions_3D = [np.array([[0.0],[0.0],[0.0]]),
                      np.array([[0.0],[0.0],[-1.0]]),
                      np.array([[0.0],[0.0],[0.0]])]


endConditions_3D = [np.array([[1000.0],[0.0],[-100.0]]),
                    np.array([[25.0],[0.0],[0.0]]),
                    np.array([[0.0],[0.0],[0.0]])]

optimizer = takeoffOptimizer(plane=plane_msg,
                             d=d,
                             M=M)

numControlPoints = 100


optimizer.getMinTimePoints(startConditions_3D=startConditions_3D,
                           endConditions_3D=endConditions_3D,
                           numVariableControlPoints=numControlPoints)



potato = 0
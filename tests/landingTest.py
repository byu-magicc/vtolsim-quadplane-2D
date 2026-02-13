import os, sys
from pathlib import Path

sys.path.insert(0, os.fspath(Path(__file__).parents[1]))

from planners.takeoffGenerator import takeoffGenerator, pathTypes
import numpy as np
from rrt_mavsim.message_types.msg_plane import MsgPlane


import matplotlib.pyplot as plt

# """
mapOrigin_2D = np.array([[0.0], [0.0]])
mapOrigin_3D = np.array([[0.0], [0.0], [0.0]])
n_hat = np.array([[0.0], [1.0], [0.0]])

plane_msg = MsgPlane(n_hat=n_hat, origin_3D=mapOrigin_3D)


rho = np.array([1.0, 1.0, 1.0])

takeoff_gen = takeoffGenerator(plane=plane_msg, rho=rho, numDimensions=2, d=3, M=10)

takeoff_gen.generatePath(
    pathType=pathTypes.PARABOLA_LANDING,
)


# """

testPoint = 0

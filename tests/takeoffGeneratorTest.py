#launch file to test the takeoff generator

import os, sys
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))


from planners.takeoffGenerator import takeoffGenerator, pathTypes
import numpy as np
from rrt_mavsim.message_types.msg_plane import MsgPlane

import matplotlib.pyplot as plt


mapOrigin_2D = np.array([[0.0],[0.0]])
mapOrigin_3D = np.array([[0.0],[0.0],[0.0]])
n_hat = np.array([[0.0],[1.0],[0.0]])

plane_msg = MsgPlane(n_hat=n_hat,
                     origin_3D=mapOrigin_3D)


rho = np.array([1.0,1.0,1.0])

takeoff_gen = takeoffGenerator(plane=plane_msg,
                               rho=rho,
                               numDimensions=2,
                               d=3,
                               M=10)




#creates the start position in 3D
startPos_3D = np.array([[0.0],[0.0],[0.0]])
startVel_3D = np.array([[0.0],[0.0],[-1.0]])
startAccel_3D = np.array([[0.0],[0.0],[-1.0]])

startConditions_3D = [startPos_3D, startVel_3D, startAccel_3D]

endPos_3D = np.array([[500.0],[0.0],[-100.0]])
endVel_3D = np.array([[25.0],[0.0],[0.0]])
endAccel_3D = np.array([[0.0],[0.0],[0.0]])

endConditions_3D = [endPos_3D, endVel_3D, endAccel_3D]


controlPoints =\
    takeoff_gen.generatePath(startPosition_3D=startPos_3D,
                         endPosition_3D=endPos_3D,
                         startVelocity=1.0,
                         endVelocity=25.0,
                         startAccel=0.0,
                         endAccel=0.0)

plt.figure(0)
plt.scatter(x=controlPoints[0,:],
            y=controlPoints[1,:])
plt.show()


potato = 0
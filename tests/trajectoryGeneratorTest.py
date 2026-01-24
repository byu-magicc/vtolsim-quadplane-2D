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



#runs through a test for the parabola function 
#That is just getting the right points off a parabola
t_list = np.linspace(0, 10, 101)
t_list.tolist()

verticesList = []

Amp=1.0
vertex = np.array([[0.0],[0.0]])

degree = 1/3

t1=0.0

t2 = gen.getTimeFromNorth(north=10.0,
                                vertex=vertex,
                                polynomialDegree=degree,
                                directionIsForward=True)


arcLength = gen.getArcLengthPolynomial(t1=t1,
                           t2=t2,
                           Amp=Amp,
                           vertex_2D=vertex,
                           polynomialDegree=degree)

t_out = gen.getTimeFromArcLength(endArcLength=arcLength,
                                 Amp=Amp,
                                 vertex=vertex,
                                 polynomialDegree=degree)


controlPoints = gen.generatePolynomial_takeoff(startConditions_3D=startConditions,
                               endConditions_3D=endConditions,
                               polynomialDegree=degree)
plt.figure(0)
plt.plot(controlPoints[0,:], controlPoints[1,:])
plt.scatter(controlPoints[0,:], controlPoints[1,:])
plt.show()

testPoint = 0

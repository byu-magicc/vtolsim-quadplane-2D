import numpy as np
import os, sys
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))

from planners.trajectoryGenerator import trajectoryGenerator, pathTypes

from rrt_mavsim.message_types.msg_plane import MsgPlane
import matplotlib.pyplot as plt

#'''
#Takeoff

straightLength = 2000.0
transitionLength = 500.0

north_startTakeoff = 0.0
north_endTakeoff = north_startTakeoff + transitionLength
north_startLanding = north_endTakeoff + straightLength
north_endLanding = north_startLanding + transitionLength

cruisingVelocity = 25.0



startConditions_takeoff = [np.array([[north_startTakeoff],[0.0],[0.0]]),
                   np.array([[0.0],[0.0],[-1.0]]),
                   np.array([[0.0],[0.0],[0.0]])]


endConditions_takeoff = [np.array([[north_endTakeoff],[0.0],[-100.0]]),
                   np.array([[cruisingVelocity],[0.0],[0.0]]),
                   np.array([[0.0],[0.0],[0.0]])]
#'''

#'''
#Landing
startConditions_landing = [np.array([[north_startLanding],[0.0],[-100.0]]),
                   np.array([[cruisingVelocity],[0.0],[0.0]]),
                   np.array([[0.0],[0.0],[0.0]])]


endConditions_landing = [np.array([[north_endLanding],[0.0],[0.0]]),
                   np.array([[0.0],[0.0],[1.0]]),
                   np.array([[0.0],[0.0],[0.0]])]
#'''

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

#'''
controlPoints_takeoff = gen.generatePolynomialTrajectory(startConditions_3D=startConditions_takeoff,
                                                 endConditions_3D=endConditions_takeoff,
                                                 polynomialDegree=degree,
                                                 directionIsForward=True)
#'''



controlPoints_landing = gen.generatePolynomialTrajectory(startConditions_3D=startConditions_landing,
                                                         endConditions_3D=endConditions_landing,
                                                         polynomialDegree=degree,
                                                         directionIsForward=False)


#gets the end takeoff and start landing control points
endTakeoffPoint = controlPoints_takeoff[:,-1:]
startLandingPoint = controlPoints_landing[:,0:1]


controlPoints_straight = gen.generateLinearInterpolation(startControlPoint=endTakeoffPoint,
                                                         endControlPoint=startLandingPoint,
                                                         velocity=cruisingVelocity)

#creates the complete concatenation
controlPoints = np.concatenate((controlPoints_takeoff,
                                 controlPoints_straight,
                                 controlPoints_landing), axis=1)

testPoint = 0

plt.figure(0)
plt.plot(controlPoints[0,:], controlPoints[1,:])
plt.show()


testPoint = 0

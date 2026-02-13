import os, sys
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))

from message_types.msg_delta import MsgDelta
import numpy as np
from models.quadplaneDynamics import QuadplaneDynamics
from rrt_mavsim.message_types.msg_plane import MsgPlane
from rrt_mavsim.tools.plane_projections_2 import *
import matplotlib.pyplot as plt


n_hat = np.array([[0.0],[-1.0],[0.0]])
origin_3D = np.array([[0.0],[0.0],[0.0]])
plane = MsgPlane(n_hat=n_hat, origin_3D=origin_3D)


numAngles = 11
angle_list = []
fn_list = []
fd_list = []
My_list = []

#gets the forces and moments for the different positions and see if they match what we expect
for i in range(numAngles):

    angle_degrees = (i-5)*5

    quad = QuadplaneDynamics(plane_msg=plane,
                             pos_3D_inertial_init=np.array([[0.0],[0.0],[0.0]]),
                             vel_3D_inertial_init=np.array([[25.0],[0.0],[0.0]]),
                             theta0=np.radians(angle_degrees),
                             q0=0.0)

    delta = MsgDelta()
    #gets the forces and moments
    forces_moments_body = quad._forces_moments(delta=delta)

    angle_list.append(angle_degrees)
    fn_list.append(forces_moments_body.item(0))
    fd_list.append(forces_moments_body.item(1))
    My_list.append(forces_moments_body.item(2))




plt.figure(0)
plt.plot(angle_list, fn_list, color='b', label='north')
plt.plot(angle_list, fd_list, color='g', label='down')
plt.legend()
plt.show()

plt.figure(1)
plt.plot(angle_list, My_list, label='moment')
plt.legend()
plt.show()


testPoint = 0

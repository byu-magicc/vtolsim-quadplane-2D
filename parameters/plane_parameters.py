#when I say plane, I am referrring to the 2D plane this thing is operating on 
#and not the actual AEROplane itself.
import numpy as np
from rrt_mavsim.message_types.msg_plane import MsgPlane

#sets the n hat vector
n_hat = np.array([[0.0],[1.0],[0.0]])
n_hat = n_hat / np.linalg.norm(n_hat)

planeOrigin = np.array([[0.0],[0.0],[0.0]])

plane_msg = MsgPlane(n_hat=n_hat,
                     origin_3D=planeOrigin)
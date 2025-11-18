import numpy as np

#creates the message class for a Plane. Used as a conversion between 2D operation and a 3D world
class MsgPlane:
    
    #Arguments:
    #1. n_hat: the normal vector to the plane, expressed in the world 3D frame
    #2. origin_3D: the origin on the map's 2D coordinate system, in the world 3D frame
    def __init__(self,
                 n_hat: np.ndarray,
                 origin_3D: np.ndarray):

        self.n_hat = n_hat
        self.origin_3D = origin_3D

    

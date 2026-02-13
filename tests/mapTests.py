import numpy as np

from rrt_mavsim.message_types.msg_world_map import MsgWorldMap, PlanarVTOLSimplifiedParams, PlanarVTOLParams, MapTypes
from rrt_mavsim.message_types.msg_plane import MsgPlane
from rrt_mavsim.tools.plane_projections import *




mapOrigin_3D = np.array([[0.0],[0.0],[0.0]])
n_hat = np.array([[0.0],[1.0],[0.0]])

msg_plane = MsgPlane(n_hat=n_hat,
                     origin_3D=mapOrigin_3D)

mapOrigin_2D = map_3D_to_2D_planeMsg(vec_3D=mapOrigin_3D,
                                     plane_msg=msg_plane)


#creates two world maps
params_1 = PlanarVTOLParams(mapOrigin_3D=mapOrigin_3D,
                            n_hat=n_hat,
                            mapOrigin_2D=mapOrigin_2D)

world_map_1 = MsgWorldMap(obstacleFieldType=MapTypes.PLANAR_VTOL,
                          numDimensions_algorithm=2,
                          planarVTOL_Params=params_1)



params_2 = PlanarVTOLSimplifiedParams(plane=msg_plane)

world_map_2 = MsgWorldMap(obstacleFieldType=MapTypes.PLANAR_VTOL_SIMPLIFIED,
                          numDimensions_algorithm=2,
                          planarVTOLSimplified_Params=params_2)



potato = 0
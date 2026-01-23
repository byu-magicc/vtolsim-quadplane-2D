import numpy as np
from rrt_mavsim.message_types.msg_plane import MsgPlane
from rrt_mavsim.tools.plane_projections import map_3D_to_2D_planeMsg
from eVTOL_BSplines.path_generation_helpers.staticFlightPath import staticFlightPath
from enum import Enum
import matplotlib.pyplot as plt



class pathTypes(str, Enum):
    LINEAR = "Linear"
    PARABOLA_TAKEOFF = "Parabola"
    PARABOLA_LANDING = "Parabola Landing"
    CUBIC = "Cubic"
    QUARTIC = "Quartic"



class trajectoryGenerator:

    def __init__(
        self,
        plane: MsgPlane,
        rho: np.ndarray,
        numDimensions: int = 2,
        d: int = 3,
        M: int = 10,
    ):
        self.plane = plane

        self.rho = rho
        self.numDimensions = numDimensions
        self.d = d
        self.M = M
        self.staticPathGenerator = staticFlightPath()



    def generateTrajectory(self,
                           path_type: pathTypes,
                           startConditions_3D: list[np.ndarray],
                           endConditions_3D: list[np.ndarray]):
        


        if path_type == pathTypes.PARABOLA_TAKEOFF:
            self.generateParabola_takeoff(startConditions_3D=startConditions_3D,
                                          endConditions_3D=endConditions_3D)



    def generateParabola_takeoff(self,
                                 startConditions_3D: list[np.ndarray],
                                 endConditions_3D: list[np.ndarray]):
        startConditions_2D = conditions_3D_to_2D(conditions_3D=startConditions_3D,
                                                 plane=self.plane) 
        endConditions_2D = conditions_3D_to_2D(conditions_3D=endConditions_3D,
                                               plane=self.plane)

        

def conditions_3D_to_2D(conditions_3D: list[np.ndarray],
                        plane: MsgPlane):
    
    conditions_2D = []
    for condition_3D in conditions_3D:

        condition_2D = map_3D_to_2D_planeMsg(vec_3D=condition_3D,
                                             plane_msg=plane)
        conditions_2D.append(condition_2D)

    return conditions_2D






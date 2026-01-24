import numpy as np
from rrt_mavsim.message_types.msg_plane import MsgPlane
from rrt_mavsim.tools.plane_projections import map_3D_to_2D_planeMsg
from eVTOL_BSplines.path_generation_helpers.staticFlightPath import staticFlightPath
from enum import Enum
import matplotlib.pyplot as plt



class pathTypes(str, Enum):
    LINEAR = "Linear"
    POLYNOMIAL_TAKEOFF = "Polynomial_Takeoff"



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
        
        self.searchResolution = 0.01
        
        self.alphaForward = 1.0
        self.alphaBackward = -1.0



    def generateTrajectory(self,
                           path_type: pathTypes,
                           startConditions_3D: list[np.ndarray],
                           endConditions_3D: list[np.ndarray],
                           polynomialDegree: float = 0.5):
        


        if path_type == pathTypes.POLYNOMIAL_TAKEOFF:
            self.generatePolynomial_takeoff(startConditions_3D=startConditions_3D,
                                            endConditions_3D=endConditions_3D,
                                            polynomialDegree=polynomialDegree)



    def generatePolynomial_takeoff(self,
                                 startConditions_3D: list[np.ndarray],
                                 endConditions_3D: list[np.ndarray],
                                 polynomialDegree: float):
        startConditions_2D = conditions_3D_to_2D(conditions_3D=startConditions_3D,
                                                 plane=self.plane) 
        endConditions_2D = conditions_3D_to_2D(conditions_3D=endConditions_3D,
                                               plane=self.plane)

        startControlPoints = self.staticPathGenerator.getLocalizedControlPoints(conditions=startConditions_2D,
                                                                                d=self.d,
                                                                                M=self.M)

        endControlPoints = self.staticPathGenerator.getLocalizedControlPoints(conditions=endConditions_2D,
                                                                                d=self.d,
                                                                                M=self.M)

        
        # gets the highest start control point (the beginning point for the parabola)
        parabolaStart = startControlPoints[:, -1:]
        # and the ending point for the parabola
        parabolaEnd = endControlPoints[:, 0:1]









    #defines the function to get a point on a parabola using parametric equations
    def getPointPolynomial(self,
                         t: float,
                         Amp: float,
                         vertex_2D: np.ndarray,
                         polynomialDegree: float,
                         directionIsForward: bool = True,
                         ):

        #if the root direction is forward (parabola opens towards the positive north axis)
        #then we set alpha to 1. If it's backwards,

        alpha = self.getAlpha(directionIsForward=directionIsForward)
        N_0 = vertex_2D[0]
        A_0 = vertex_2D[1]

        north =(t**(1/polynomialDegree))/alpha + N_0
        altitude = Amp*t + A_0

        position = np.array([[north],[altitude]])

        return position
    
    def getArcLengthPolynomial(self,
                             t1: float,
                             t2: float,
                             Amp: float,
                             vertex_2D: np.ndarray,
                             polynomialDegree: float,
                             directionIsForward: bool = True)->float:

        numSteps = int((t2 - t1) / self.searchResolution)
        
        
        arcLength = 0.0

        for i in range(numSteps):

            currentTime = t1 + i * self.searchResolution
            nextTime = t1 + (i + 1) * self.searchResolution


            currentPosition = self.getPointPolynomial(t=currentTime,
                                                    Amp=Amp,
                                                    vertex_2D=vertex_2D,
                                                    polynomialDegree=polynomialDegree,
                                                    directionIsForward=directionIsForward)

            nextPosition = self.getPointPolynomial(t=nextTime,
                                                 Amp=Amp,
                                                 vertex_2D=vertex_2D,
                                                 polynomialDegree=polynomialDegree,
                                                 directionIsForward=directionIsForward)

            distance = np.linalg.norm(nextPosition - currentPosition)
            
            arcLength += distance

        #returns the arc length
        return arcLength

    def getTimeFromArcLength(self,
                           endArcLength: float,
                           Amp: float,
                           vertex: np.ndarray,
                           polynomialDegree: float,
                           directionIsForward: bool = True)->float:

        currentTime = 0.0
        counter = 0

        currentArcLength = 0.0
        while currentArcLength < endArcLength:

            currentTime = counter*self.searchResolution 
            nextTime = (counter+1)*self.searchResolution


            currentPosition = self.getPointPolynomial(t=currentTime,
                                                    Amp=Amp,
                                                    vertex_2D=vertex,
                                                    polynomialDegree=polynomialDegree,
                                                    directionIsForward=directionIsForward)

            nextPosition = self.getPointPolynomial(t=nextTime,
                                                 Amp=Amp,
                                                 vertex_2D=vertex,
                                                 polynomialDegree=polynomialDegree,
                                                 directionIsForward=directionIsForward)



            tempDistance = np.linalg.norm(nextPosition - currentPosition)
            
            currentArcLength += tempDistance
        
            counter += 1

        return nextTime
        


        




    def getTimeFromNorth(self,
                         north: float,
                         vertex: np.ndarray,
                         polynomialDegree: float,
                         directionIsForward: bool = True):

        alpha = self.getAlpha(directionIsForward=directionIsForward)

        
        N_0 = vertex.item(0)
        
        t = (alpha*(north-N_0))**polynomialDegree


        return t


    def getTimeFromAltitude(self,
                            altitude: float,
                            vertex: np.ndarray,
                            Amp: float):

        A_0 = vertex.item(1)
        t = (altitude - A_0)/Amp

        return t

    def getAlpha(self,
                 directionIsForward: bool = True):

        if directionIsForward:
            alpha = self.alphaForward
        else:
            alpha = self.alphaBackward
        return alpha

        

def conditions_3D_to_2D(conditions_3D: list[np.ndarray],
                        plane: MsgPlane):
    
    conditions_2D = []
    for condition_3D in conditions_3D:

        condition_2D = map_3D_to_2D_planeMsg(vec_3D=condition_3D,
                                             plane_msg=plane)
        conditions_2D.append(condition_2D)

    return conditions_2D






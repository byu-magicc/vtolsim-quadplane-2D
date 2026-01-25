
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
            self.generatePolynomialTrajectory(startConditions_3D=startConditions_3D,
                                            endConditions_3D=endConditions_3D,
                                            polynomialDegree=polynomialDegree)

    def generateCompleteTrajectory(self,
                                   startConditions_takeoff: list[np.ndarray],
                                   endConditions_takeoff: list[np.ndarray],
                                   startConditions_landing: list[np.ndarray],
                                   endConditions_landing: list[np.ndarray],
                                   polynomialDegree: float):

        #gets the control points for the takeoff
        takeoffControlPoints = self.generatePolynomialTrajectory(startConditions_3D=startConditions_takeoff,
                                                                 endConditions_3D=endConditions_takeoff,
                                                                 polynomialDegree=polynomialDegree,
                                                                 directionIsForward=True)

        landingControlPoints = self.generatePolynomialTrajectory(startConditions_3D=startConditions_landing,
                                                                 endConditions_3D=endConditions_landing,
                                                                 polynomialDegree=polynomialDegree,
                                                                 directionIsForward=False)
        
        #gets the norm of the velocity of the end conditions takeoff
        velocity_cruise = np.linalg.norm(endConditions_takeoff[1])
        endTakeoffPoint = takeoffControlPoints[:,-1:]

        startLandingPoint = landingControlPoints[:,0:1]

        #gets the interpolation straight control points
        straightControlPoints = self.generateLinearInterpolation(startControlPoint=endTakeoffPoint,
                                                                 endControlPoint=startLandingPoint,
                                                                 velocity=velocity_cruise)

        #concatenates together the control points
        controlPoints = np.concatenate((takeoffControlPoints, straightControlPoints, landingControlPoints), axis=1)
        

        testPoint = 0


        return controlPoints

    def generatePolynomialTrajectory(self,
                                 startConditions_3D: list[np.ndarray],
                                 endConditions_3D: list[np.ndarray],
                                 polynomialDegree: float,
                                 directionIsForward: bool = True):

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

        #case the direction is Forward, in which case the parabola starts from the start point
        if directionIsForward:
        
            # gets the highest start control point (the beginning point for the parabola)
            polynomialStartPoint = startControlPoints[:, -1:]
            # and the ending point for the parabola
            polynomialEndPoint = endControlPoints[:, 0:1]
            #gets the initial velocity at the vertex
            v_init = np.linalg.norm(startConditions_2D[1])
            v_end = np.linalg.norm(endConditions_2D[1])

            #gets the change in velocity
            delta_velocity = v_end - v_init

            testPoint = 0

        else:

            # gets the highest start control point (the beginning point for the parabola)
            polynomialStartPoint = endControlPoints[:, 0:1]
            # and the ending point for the parabola
            polynomialEndPoint = startControlPoints[:, -1:]

            #gets the initial velocity at the vertex
            v_init = np.linalg.norm(endConditions_2D[1])
            v_end = np.linalg.norm(startConditions_2D[1])

            #gets the change in velocity
            delta_velocity = v_end - v_init
            testPoint = 0

        alpha = self.getAlpha(directionIsForward=directionIsForward)

        #gets the change in north position
        delta_north = polynomialEndPoint.item(0) - polynomialStartPoint.item(0)
        delta_altitude = polynomialEndPoint.item(1) - polynomialStartPoint.item(1)

        north_end = polynomialEndPoint.item(0)
        
        #gets the time for that change in north position
        t_end = self.getTimeFromNorth(north=north_end,
                                      vertex=polynomialStartPoint,
                                      polynomialDegree=polynomialDegree,
                                      directionIsForward=directionIsForward)


        #gets the amplitude
        Amp = delta_altitude / ((alpha*delta_north)**polynomialDegree)



        #gets the total arc length
        totalArcLength = self.getArcLengthPolynomial(t1=0.0,
                                                     t2=t_end,
                                                     Amp=Amp,
                                                     vertex_2D=polynomialStartPoint,
                                                     polynomialDegree=polynomialDegree)

        spatialAcceleration = delta_velocity / totalArcLength

        
        #goes through to space out the control points
        controlPoints_list = []
        times_list = []
        #creates the current arc length
        currentArcLength = 0.0
        #flag for whether we are at the end
        withinEnd = False

        while not withinEnd:


            #gets the current desired velocity
            currentVel_desired = v_init + spatialAcceleration*currentArcLength

            currentArcLength += currentVel_desired 

            #gets the arc time from the length
            currentTime = self.getTimeFromArcLength(endArcLength=currentArcLength,
                                                    Amp=Amp,
                                                    vertex=polynomialStartPoint,
                                                    polynomialDegree=polynomialDegree)
            times_list.append(currentTime)

            currentPosition = self.getPointPolynomial(t=currentTime,
                                                      Amp=Amp,
                                                      vertex_2D=polynomialStartPoint,
                                                      polynomialDegree=polynomialDegree,
                                                      directionIsForward=directionIsForward)

            controlPoints_list.append(currentPosition)
            

            # if the current arc length is within the desired velocity's length of the end
            if (totalArcLength - currentArcLength) < currentVel_desired:
                withinEnd = True


        #case it is open backwards
        if not directionIsForward:
            controlPoints_list.reverse()

        #concatenates the control points
        controlPoints = np.concatenate(controlPoints_list, axis=1)
        shapeMainList = np.shape(controlPoints)
        shapeStartList = np.shape(startControlPoints)
        shapeEndList = np.shape(endControlPoints)

        #concatenates on the start and end control points 
        controlPoints = np.concatenate((startControlPoints, controlPoints, endControlPoints), axis=1)
        

        return controlPoints

    #generates a linear interpolation between two control points with a constant velocity
    def generateLinearInterpolation(self,
                                    startControlPoint: np.ndarray,
                                    endControlPoint: np.ndarray,
                                    velocity: float):
        
        differenceVector = endControlPoint - startControlPoint
        #gets the total distance
        distanceTotal = np.linalg.norm(differenceVector)
        #gets the direction vector
        direction_hat = differenceVector / distanceTotal 
        

        #gets the distance divided by velocity
        numControlSpaces = int(distanceTotal / velocity)

        #gets the actual interpolation velocity so that there's no awkward spaces
        actualVelocity = distanceTotal / numControlSpaces
        actualVelocity_vector = actualVelocity * direction_hat
    
        #the number of control points is the number of spaces minus one
        numControlPoints = numControlSpaces - 1

        controlPointsList = []

        for i in range(numControlPoints):
            currentPosition = startControlPoint + (i+1)*actualVelocity_vector
            controlPointsList.append(currentPosition)

        controlPoints = np.concatenate((controlPointsList), axis=1)

        return controlPoints

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
        N_0 = vertex_2D.item(0)
        A_0 = vertex_2D.item(1)

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






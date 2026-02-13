# creates the generator which creates the conditions necessary for takeoff
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


class parabolaDirection(str, Enum):
    OPENS_FORWARD = "Forward"
    OPENS_BACKWARD = "Backward"


class flightPathGenerator:
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

        # sets the search resolution
        self.searchResolution = 0.1

        self.alpha_takeoff = 1.0
        self.alpha_landing = -1.0

    def generatePath(
        self,
        startConditions_3D: list[np.ndarray],
        endConditions_3D: list[np.ndarray],
        alpha: float = 1.0,
        pathType: pathTypes = pathTypes.PARABOLA_TAKEOFF,
    )->np.ndarray:

        #initializes the control points array
        controlPoints = np.array([[0.0]])
        match pathType:
            case pathTypes.PARABOLA_TAKEOFF:
                controlPoints = self.generateParabolicTakeoffPath(
                    startConditions_3D=startConditions_3D,
                    endConditions_3D=endConditions_3D,
                )

            case pathTypes.PARABOLA_LANDING:
                controlPoints = self.generateParabolicLandingPath(
                    startConditions_3D=startConditions_3D,
                    endConditions_3D=endConditions_3D,
                )
        return controlPoints
    # the start and end positions are given in coordinates (In 3D)
    # but the velocities are given in magnitude for easier tunin
    def generateParabolicTakeoffPath(
        self,
        startConditions_3D: list[np.ndarray],
        endConditions_3D: list[np.ndarray],
    ):
        # gets the positions in 2D
        startPosition_2D = map_3D_to_2D_planeMsg(
            vec_3D=startConditions_3D[0], plane_msg=self.plane
        )
        endPosition_2D = map_3D_to_2D_planeMsg(
            vec_3D=endConditions_3D[0], plane_msg=self.plane
        )


        startVel_2D = map_3D_to_2D_planeMsg(vec_3D=startConditions_3D[1], plane_msg=self.plane)
        endVel_2D = map_3D_to_2D_planeMsg(vec_3D=endConditions_3D[1], plane_msg=self.plane)

        startAccel_2D = map_3D_to_2D_planeMsg(
            vec_3D=startConditions_3D[2], plane_msg=self.plane
        )
        endAccel_2D = map_3D_to_2D_planeMsg(vec_3D=endConditions_3D[2], plane_msg=self.plane)

        # start and end conditions
        startConditions_2D = [startPosition_2D, startVel_2D, startAccel_2D]
        endConditions_2D = [endPosition_2D, endVel_2D, endAccel_2D]

        # gets the initial and final control points
        startControlPoints = self.staticPathGenerator.getLocalizedControlPoints(
            conditions=startConditions_2D, d=self.d, M=self.M
        )

        endControlPoints = self.staticPathGenerator.getLocalizedControlPoints(
            conditions=endConditions_2D, d=self.d, M=self.M
        )

        # gets the highest start control point (the beginning point for the parabola)
        parabolaStart = startControlPoints[:, -1:]
        # and the ending point for the parabola
        parabolaEnd = endControlPoints[:, 0:1]

        # gets the change in altitude and North
        delta_north = parabolaEnd.item(0) - parabolaStart.item(0)
        delta_altitude = parabolaEnd.item(1) - parabolaStart.item(1)

        #gets the time corresponding to the north position
        t_end = np.sqrt(self.alpha_takeoff*(delta_north))

        Amp = delta_altitude / np.sqrt(self.alpha_takeoff*delta_north)

        # gets the initial spacing of control points
        centerControlPoint_start = startControlPoints[:, 1:2]
        centerControlPoint_end = endControlPoints[:, 1:2]

        # gets the initial and final spacing of control points
        initialVelocity = np.linalg.norm(parabolaStart - centerControlPoint_start)
        finalVelocity = np.linalg.norm(centerControlPoint_end - parabolaEnd)

        # gets the change in spacing
        delta_velocity = finalVelocity - initialVelocity

        # gets the total arc length
        totalArcLength = self.getArcLengthParabola(
            t1=0.0, t2=t_end, Amp=Amp, vertex_2D=parabolaStart, alpha=self.alpha_takeoff
        )

        controlPoints_list = []

        # spatial acceleration over the length
        spatialAcceleration = delta_velocity / totalArcLength

        # initializes the current arc length as the initial velocity
        currentArcLength = 0.0
        currentTime_list = []

        withinEnd = False

        while not withinEnd:
            # gets the current desired velocity
            current_desired_velocity = (
                initialVelocity + currentArcLength * spatialAcceleration
            )

            # changes the current arc length to include a new section that's as long as the current desired velocity
            currentArcLength += current_desired_velocity

            # gets the time for the current arc length
            currentTime = self.getArcPositionTime(
                endArcLength=currentArcLength,
                Amp=Amp,
                vertex_2D=startPosition_2D,
                alpha=self.alpha_takeoff,
            )

            currentTime_list.append(currentTime)

            # gets the actual position for this current time
            currentPosition, _ = self.getPointParabola(
                t=currentTime, Amp=Amp, vertex_2D=startPosition_2D, alpha=self.alpha_takeoff
            )

            controlPoints_list.append(currentPosition)

            # if the current arc length is within the desired velocity's length of the end
            if (totalArcLength - currentArcLength) < current_desired_velocity:
                withinEnd = True
        

        # control points as an 2D array
        controlPoints = np.concatenate((controlPoints_list), axis=1)

        # concatenates on the start and end control points
        controlPoints = np.concatenate((startControlPoints, controlPoints), axis=1)
        controlPoints = np.concatenate((controlPoints, endControlPoints), axis=1)

        # returns the control point positions
        return controlPoints
    #'''



    # the start and end positions are given in coordinates (In 3D)
    # but the velocities are given in magnitude for easier tunin
    def generateParabolicLandingPath(
        self,
        startConditions_3D: list[np.ndarray],
        endConditions_3D: list[np.ndarray],
    ):
        # gets the positions in 2D
        startPosition_2D = map_3D_to_2D_planeMsg(
            vec_3D=startConditions_3D[0], plane_msg=self.plane
        )
        endPosition_2D = map_3D_to_2D_planeMsg(
            vec_3D=endConditions_3D[0], plane_msg=self.plane
        )


        startVel_2D = map_3D_to_2D_planeMsg(vec_3D=startConditions_3D[1], plane_msg=self.plane)
        endVel_2D = map_3D_to_2D_planeMsg(vec_3D=endConditions_3D[1], plane_msg=self.plane)

        startAccel_2D = map_3D_to_2D_planeMsg(
            vec_3D=startConditions_3D[2], plane_msg=self.plane
        )
        endAccel_2D = map_3D_to_2D_planeMsg(vec_3D=endConditions_3D[2], plane_msg=self.plane)

        # start and end conditions
        startConditions_2D = [startPosition_2D, startVel_2D, startAccel_2D]
        endConditions_2D = [endPosition_2D, endVel_2D, endAccel_2D]

        # gets the initial and final control points
        startControlPoints = self.staticPathGenerator.getLocalizedControlPoints(
            conditions=startConditions_2D, d=self.d, M=self.M
        )

        endControlPoints = self.staticPathGenerator.getLocalizedControlPoints(
            conditions=endConditions_2D, d=self.d, M=self.M
        )

        # gets the highest start control point (the beginning point for the parabola)
        parabolaStart = endControlPoints[:, 0:1]
        # and the ending point for the parabola
        parabolaEnd = startControlPoints[:, -1:]

        # gets the change in altitude and North
        delta_north = parabolaEnd.item(0) - parabolaStart.item(0)
        delta_altitude = parabolaEnd.item(1) - parabolaStart.item(1)

        #gets the time corresponding to the north position
        t_end = np.sqrt(np.abs(self.alpha_landing*(delta_north)))

        Amp = delta_altitude / np.sqrt(abs(self.alpha_landing*delta_north))

        # gets the initial spacing of control points
        centerControlPoint_start = startControlPoints[:, 1:2]
        centerControlPoint_end = endControlPoints[:, 1:2]

        # gets the initial and final spacing of control points
        initialVelocity = np.linalg.norm(parabolaEnd - centerControlPoint_start)
        finalVelocity = np.linalg.norm(centerControlPoint_end - parabolaStart)

        # gets the change in spacing
        delta_velocity = finalVelocity - initialVelocity

        # gets the total arc length
        totalArcLength = self.getArcLengthParabola(
            t1=0.0, t2=t_end, Amp=Amp, vertex_2D=parabolaStart, alpha=self.alpha_landing
        )

        controlPoints_list = []

        # spatial acceleration over the length
        spatialAcceleration = abs(delta_velocity / totalArcLength)

        # initializes the current arc length as the initial velocity
        currentArcLength = 0.0

        withinEnd = False

        while not withinEnd:
            # gets the current desired velocity
            current_desired_velocity = (
                finalVelocity + currentArcLength * spatialAcceleration
            )

            # changes the current arc length to include a new section that's as long as the current desired velocity
            currentArcLength += current_desired_velocity

            # gets the time for the current arc length
            currentTime = self.getArcPositionTime(
                endArcLength=currentArcLength,
                Amp=Amp,
                vertex_2D=parabolaStart,
                alpha=self.alpha_landing,
            )

            # gets the actual position for this current time
            currentPosition, _ = self.getPointParabola(
                t=currentTime, Amp=Amp, vertex_2D=parabolaStart, alpha=self.alpha_landing
            )



            controlPoints_list.append(currentPosition)

            # if the current arc length is within the desired velocity's length of the end
            if (totalArcLength - currentArcLength) < current_desired_velocity:
                withinEnd = True


        #flips the order of the control points
        controlPoints_list.reverse()
        # control points as an 2D array
        controlPoints = np.concatenate((controlPoints_list), axis=1)

        # concatenates on the start and end control points
        controlPoints = np.concatenate((startControlPoints, controlPoints), axis=1)
        controlPoints = np.concatenate((controlPoints, endControlPoints), axis=1)

        # returns the control point positions
        return controlPoints
    #'''



    # defines the function to get the individual points for a specific time
    # t: time value for point
    # Amp: the amplitude of the wave
    # vertex_2D: the point of the vertex of the sqrt function
    # alpha: the time scaling factor
    def getPointParabola(
        self,
        t: float,
        Amp: float,
        vertex_2D: np.ndarray,
        alpha: float = 1.0,
    ):
        # gets the start altitude and north position
        N_0 = vertex_2D.item(0)
        A_0 = vertex_2D.item(1)
        
        #gets the north position as a function of time
        north =(t**2)/alpha + N_0

        # gets the altitude as a function of time
        #altitude = np.sqrt((alpha * t - N_0) / c) + A_0
        altitude = Amp*t + A_0

        # creates the array for the position
        output_position_2D = np.array([[north], [altitude]])

        # case we are at the vertex
        if t == 0.0:
            tangentVector = np.array([[0.0], [Amp]])
            tangentVector = tangentVector / np.linalg.norm(tangentVector)
        else:
            d_altitude_d_north = (Amp*alpha)/(2*t)
            tangentVector = np.array([[1], [d_altitude_d_north]])
            # and then normalizes it
            tangentVector = tangentVector / np.linalg.norm(tangentVector)

            testPoint = 0

        # section to get the velocity and acceleration magnitude

        return output_position_2D, tangentVector
    


    def getArcPositionTime(
            self, endArcLength, Amp: float,  alpha: float, vertex_2D: np.ndarray):
        currentArcLength = 0.0

        counter = 0
        while currentArcLength < endArcLength:
            currentTime = counter * self.searchResolution
            nextTime = (counter + 1) * self.searchResolution

            currentPosition, _ = self.getPointParabola(
                t=currentTime, Amp=Amp, alpha=alpha, vertex_2D=vertex_2D
            )

            nextPosition, _ = self.getPointParabola(
                t=nextTime, Amp=Amp, alpha=alpha, vertex_2D=vertex_2D,
            )

            # gets the distance between the current and the next position
            distanceVector = nextPosition - currentPosition
            tempDistance = np.linalg.norm(distanceVector)

            currentArcLength += tempDistance

            counter += 1

        # returns the next time
        return nextTime

    # function to get the arc length of the parabola between the two points
    def getArcLengthParabola(
        self, t1: float, t2: float, Amp: float, vertex_2D: np.ndarray, alpha: float
    ):
        numSteps = int((t2 - t1) / self.searchResolution)

        arcLength = 0.0

        for i in range(numSteps):
            currentTime = t1 + i * self.searchResolution
            nextTime = t1 + (i + 1) * self.searchResolution

            currentPosition, _ = self.getPointParabola(
                t=currentTime, Amp=Amp, vertex_2D=vertex_2D, alpha=alpha
            )

            nextPosition, _ = self.getPointParabola(
                t=nextTime, Amp=Amp, vertex_2D=vertex_2D, alpha=alpha
            )

            # gets the distance between the current and the next position
            distanceVector = nextPosition - currentPosition
            tempDistance = np.linalg.norm(distanceVector)

            arcLength += tempDistance

        return arcLength



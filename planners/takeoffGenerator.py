# creates the generator which creates the conditions necessary for takeoff
import numpy as np
from rrt_mavsim.message_types.msg_plane import MsgPlane
from rrt_mavsim.tools.plane_projections import map_3D_to_2D_planeMsg
from eVTOL_BSplines.path_generation_helpers.staticFlightPath import staticFlightPath
from enum import Enum


class pathTypes(str, Enum):
    LINEAR = "Linear"
    PARABOLA = "Parabola"
    PARABOLA_LANDING = "Parabola Landing"
    CUBIC = "Cubic"
    QUARTIC = "Quartic"


class parabolaDirection(str, Enum):
    OPENS_FORWARD = "Forward"
    OPENS_BACKWARD = "Backward"


class takeoffGenerator:
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

    def generatePath(
        self,
        pathType: pathTypes = pathTypes.PARABOLA,
        startPosition_3D: np.ndarray = None,
        endPosition_3D: np.ndarray = None,
        startVelocity: float = None,
        endVelocity: float = None,
        startAccel: float = None,
        endAccel: float = None,
    ):
        match pathType:
            case pathTypes.PARABOLA:
                return self.generateParabolicPath(
                    startPosition_3D=startPosition_3D,
                    endPosition_3D=endPosition_3D,
                    startVelocity=startVelocity,
                    endVelocity=endVelocity,
                    startAcceleration=startAccel,
                    endAcceleration=endAccel,
                )

            case pathTypes.PARABOLA_LANDING:
                return self.generateParabolicLandingPath(
                    startPosition_3D=startPosition_3D,
                    endPosition_3D=endPosition_3D,
                    startVelocity=startVelocity,
                    endVelocity=endVelocity,
                    startAcceleration=startAccel,
                    endAcceleration=endAccel,
                )

    def generateParabolicLandingPath(
        self,
        startPosition_3D: np.ndarray,
        endPosition_3D: np.ndarray,
        startVelocity: float,
        endVelocity: float,
        startAcceleration: float,
        endAcceleration: float,
    ):
        startPosition_2D = map_3D_to_2D_planeMsg(
            vec_3D=startPosition_3D, plane_msg=self.plane
        )

        endPosition_2D = map_3D_to_2D_planeMsg(
            vec_3D=endPosition_3D, plane_msg=self.plane
        )

        # creates the start and end velocities and accelerations
        startVel_3D = np.array([[startVelocity], [0.0], [0.0]])
        endVel_3D = np.array([[0.0], [0.0], [endVelocity]])

        startAccel_3D = np.array([[startAcceleration], [0.0], [0.0]])
        endAccel_3D = np.array([[0.0], [0.0], [endAcceleration]])

        startVel_2D = map_3D_to_2D_planeMsg(vec_3D=startVel_3D, plane_msg=self.plane)
        endVel_2D = map_3D_to_2D_planeMsg(vec_3D=endVel_3D, plane_msg=self.plane)

        startAccel_2D = map_3D_to_2D_planeMsg(
            vec_3D=startAccel_3D, plane_msg=self.plane
        )
        endAccel_2D = map_3D_to_2D_planeMsg(vec_3D=endAccel_3D, plane_msg=self.plane)

        # start and end conditions
        startConditions = [startPosition_2D, startVel_2D, startAccel_2D]
        endConditions = [endPosition_2D, endVel_2D, endAccel_2D]

        # gets the initial and final control points
        startControlPoints = self.staticPathGenerator.getLocalizedControlPoints(
            conditions=startConditions, d=self.d, M=self.M
        )

        endControlPoints = self.staticPathGenerator.getLocalizedControlPoints(
            conditions=endConditions, d=self.d, M=self.M
        )

        # gets the highest start control point (the beginning point for the parabola)
        parabolaStart = startControlPoints[:, -1:]
        # and the ending point for the parabola
        parabolaEnd = endControlPoints[:, 0:1]

        # gets the change in altitude and North
        delta_north = parabolaEnd.item(0) - parabolaStart.item(0)
        delta_altitude = parabolaEnd.item(1) - parabolaStart.item(1)

        # gets the scaling factor C
        c = delta_north / (delta_altitude**2)

        # gets the initial spacing of control points
        centerControlPoint_start = startControlPoints[:, 1:2]
        centerControlPoint_end = endControlPoints[:, 1:2]

        # gets the initial and final spacing of control points
        initialVelocity = np.linalg.norm(parabolaStart - centerControlPoint_start)
        finalVelocity = np.linalg.norm(centerControlPoint_end - parabolaEnd)

        # gets the change in spacing
        delta_velocity = finalVelocity - initialVelocity

        testPoint = 0

    # the start and end positions are given in coordinates (In 3D)
    # but the velocities are given in magnitude for easier tunin
    def generateParabolicPath(
        self,
        startPosition_3D: np.ndarray,
        endPosition_3D: np.ndarray,
        startVelocity: float,
        endVelocity: float,
        startAcceleration: float,
        endAcceleration: float,
    ):
        # gets the positions in 2D
        startPosition_2D = map_3D_to_2D_planeMsg(
            vec_3D=startPosition_3D, plane_msg=self.plane
        )
        endPosition_2D = map_3D_to_2D_planeMsg(
            vec_3D=endPosition_3D, plane_msg=self.plane
        )

        startVel_3D = np.array([[0.0], [0.0], [-startVelocity]])
        endVel_3D = np.array([[endVelocity], [0.0], [0.0]])
        startAccel_3D = np.array([[0.0], [0.0], [-startAcceleration]])
        endAccel_3D = np.array([[endAcceleration], [0.0], [0.0]])

        startVel_2D = map_3D_to_2D_planeMsg(vec_3D=startVel_3D, plane_msg=self.plane)
        endVel_2D = map_3D_to_2D_planeMsg(vec_3D=endVel_3D, plane_msg=self.plane)

        startAccel_2D = map_3D_to_2D_planeMsg(
            vec_3D=startAccel_3D, plane_msg=self.plane
        )
        endAccel_2D = map_3D_to_2D_planeMsg(vec_3D=endAccel_3D, plane_msg=self.plane)

        # start and end conditions
        startConditions = [startPosition_2D, startVel_2D, startAccel_2D]
        endConditions = [endPosition_2D, endVel_2D, endAccel_2D]

        # gets the initial and final control points
        startControlPoints = self.staticPathGenerator.getLocalizedControlPoints(
            conditions=startConditions, d=self.d, M=self.M
        )

        endControlPoints = self.staticPathGenerator.getLocalizedControlPoints(
            conditions=endConditions, d=self.d, M=self.M
        )

        # gets the highest start control point (the beginning point for the parabola)
        parabolaStart = startControlPoints[:, -1:]
        # and the ending point for the parabola
        parabolaEnd = endControlPoints[:, 0:1]

        # gets the change in altitude and North
        delta_north = parabolaEnd.item(0) - parabolaStart.item(0)
        delta_altitude = parabolaEnd.item(1) - parabolaStart.item(1)

        # gets the scaling factor C
        c = delta_north / (delta_altitude**2)

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
            t1=0.0, t2=delta_north, c=c, vertex_2D=parabolaStart, alpha=1.0
        )

        controlPoints_list = []

        # spatial acceleration over the length
        spatialAcceleration = delta_velocity / totalArcLength

        # initializes the current arc length as the initial velocity
        currentArcLength = 0.0

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
                c=c,
                vertex_2D=startPosition_2D,
                alpha=1.0,
            )

            # gets the actual position for this current time
            currentPosition, _ = self.getPointParabola(
                t=currentTime, c=c, vertex_2D=startPosition_2D, alpha=1.0
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

    # defines the function to get the individual points for a specific time
    def getPointParabola(
        self,
        t: float,
        c: float,
        vertex_2D: np.ndarray,
        alpha: float = 1.0,
        Direction: parabolaDirection = parabolaDirection.OPENS_FORWARD,
    ):
        # gets the start altitude and north position
        N_0 = vertex_2D.item(0)
        A_0 = vertex_2D.item(1)

        # gets the north position as a function of time
        north = alpha * t

        # gets the altitude as a function of time
        altitude = np.sqrt((alpha * t - N_0) / c) + A_0

        # creates the array for the position
        output_position_2D = np.array([[north], [altitude]])

        # case we are at the vertex
        if t == 0.0:
            tangentVector = np.array([[0.0], [1.0]])
        else:
            north_dot = alpha
            altitude_dot = (alpha / (2 * c)) * ((alpha / c) * t - N_0 / c) ** (-1 / 2)
            tangentVector = np.array([[north_dot], [altitude_dot]])
            # and then normalizes it
            tangentVector = tangentVector / np.linalg.norm(tangentVector)

        # section to get the velocity and acceleration magnitude

        return output_position_2D, tangentVector

    def getArcPositionTime(
        self, endArcLength: float, c: float, vertex_2D: np.ndarray, alpha: float
    ):
        currentArcLength = 0.0

        counter = 0
        while currentArcLength < endArcLength:
            currentTime = counter * self.searchResolution
            nextTime = (counter + 1) * self.searchResolution

            currentPosition, _ = self.getPointParabola(
                t=currentTime, c=c, vertex_2D=vertex_2D, alpha=alpha
            )

            nextPosition, _ = self.getPointParabola(
                t=nextTime, c=c, vertex_2D=vertex_2D, alpha=alpha
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
        self, t1: float, t2: float, c: float, vertex_2D: np.ndarray, alpha: float
    ):
        numSteps = int((t2 - t1) / self.searchResolution)

        arcLength = 0.0

        for i in range(numSteps):
            currentTime = t1 + i * self.searchResolution
            nextTime = t1 + (i + 1) * self.searchResolution

            currentPosition, _ = self.getPointParabola(
                t=currentTime, c=c, vertex_2D=vertex_2D, alpha=alpha
            )

            nextPosition, _ = self.getPointParabola(
                t=nextTime, c=c, vertex_2D=vertex_2D, alpha=alpha
            )

            # gets the distance between the current and the next position
            distanceVector = nextPosition - currentPosition
            tempDistance = np.linalg.norm(distanceVector)

            arcLength += tempDistance

        return arcLength


class takeoffGenerator_old:
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

        # creates the static flight path object
        self.staticPathGenerator = staticFlightPath()
        pass

    def generatePath(
        self,
        pathType: pathTypes = pathTypes.PARABOLA,
        numTransitions: int = None,
        startPosition_3D: np.ndarray = None,
        endPosition_3D: np.ndarray = None,
        startVelocity: float = None,
        endVelocity: float = None,
        startAccel: float = None,
        endAccel: float = None,
    ):
        match pathType:
            case pathTypes.PARABOLA:
                return self.generateParabolicPath(
                    numTransitions=numTransitions,
                    startPosition_3D=startPosition_3D,
                    endPosition_3D=endPosition_3D,
                    startVelocity=startVelocity,
                    endVelocity=endVelocity,
                    startAccel=startAccel,
                    endAccel=endAccel,
                )

    # the start and end positions are given in coordinates (In 3D)
    # but the velocities are given in magnitude for easier tunin
    def generateParabolicPath(
        self,
        numTransitions: int,
        startPosition_3D: np.ndarray,
        endPosition_3D: np.ndarray,
        startVelocity: float,
        endVelocity: float,
        startAccel: float,
        endAccel: float,
    ):
        # gets the positions in 2D
        startPosition_2D = map_3D_to_2D_planeMsg(
            vec_3D=startPosition_3D, plane_msg=self.plane
        )
        endPosition_2D = map_3D_to_2D_planeMsg(
            vec_3D=endPosition_3D, plane_msg=self.plane
        )

        # gets the start altitude and north position
        N_0 = startPosition_2D.item(0)
        A_0 = startPosition_2D.item(1)

        # the end altitude and north positions
        N_1 = endPosition_2D.item(0)
        A_1 = endPosition_2D.item(1)

        # gets the scaling factor C
        c = (N_1 - N_0) / ((A_1 - A_0) ** 2)

        # gets the change in altitude and north position
        delta_north = N_1 - N_0
        delta_altitude = A_1 - A_0

        # gets the change in velocity
        deltaVelocity = endVelocity - startVelocity

        transitionSize_velocity = deltaVelocity / numTransitions

        # gets the changes in north
        transitionSize_position = delta_north / numTransitions

        position_list = []
        tangent_list = []
        velocity_list = []
        acceleration_list = []
        condition_list = []

        # iterates over all of the transition points from start to finish
        for i in range(numTransitions + 1):
            current_time = transitionSize_position * i
            # gets the points and derivatives of the prarabola
            position, tangent = self.getPointParabola(
                t=current_time, alpha=1.0, c=c, vertex_2D=startPosition_2D
            )

            # gets the current velocity
            velMagntiude = i * transitionSize_velocity

            if i == 0:
                accelMagnitude = startAccel
            else:
                accelMagnitude = 0.0

            # gets the velocity and acceleration
            velocity = tangent * velMagntiude
            acceleration = tangent * accelMagnitude

            position_list.append(position)
            tangent_list.append(tangent)
            velocity_list.append(velocity)
            acceleration_list.append(acceleration)

            tempCondition = [position, velocity, acceleration]
            condition_list.append(tempCondition)

        # gets the control points
        controlPoints = self.staticPathGenerator.getControlPoints_conditionsList(
            conditions_list=condition_list,
            rho=self.rho,
            numDimensions=self.numDimensions,
            d=self.d,
            M=self.M,
        )

        return controlPoints

    # defines the function to get the individual points for a specific time
    def getPointParabola(
        self, t: float, c: float, vertex_2D: np.ndarray, alpha: float = 1.0
    ):
        # gets the start altitude and north position
        N_0 = vertex_2D.item(0)
        A_0 = vertex_2D.item(1)

        # gets the north position as a function of time
        north = alpha * t

        # gets the altitude as a function of time
        altitude = np.sqrt((alpha * t - N_0) / c) + A_0

        # creates the array for the position
        output_position_2D = np.array([[north], [altitude]])

        # case we are at the vertex
        if t == 0.0:
            tangentVector = np.array([[0.0], [1.0]])
        else:
            north_dot = alpha
            altitude_dot = (alpha / (2 * c)) * ((alpha / c) * t - N_0 / c) ** (-1 / 2)
            tangentVector = np.array([[north_dot], [altitude_dot]])
            # and then normalizes it
            tangentVector = tangentVector / np.linalg.norm(tangentVector)

        # section to get the velocity and acceleration magnitude

        return output_position_2D, tangentVector

    def generateCubicPath(
        self,
        numTransitions: int,
        startPosition_3D: np.ndarray,
        endPosition_3D: np.ndarray,
        startVelocity: float,
        endVelocity: float,
        startAccel: float,
        endAccel: float,
    ):
        # gets the positions in 2D
        startPosition_2D = map_3D_to_2D_planeMsg(
            vec_3D=startPosition_3D, plane_msg=self.plane
        )
        endPosition_2D = map_3D_to_2D_planeMsg(
            vec_3D=endPosition_3D, plane_msg=self.plane
        )

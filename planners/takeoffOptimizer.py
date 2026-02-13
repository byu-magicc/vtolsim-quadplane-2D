import numpy as np
from rrt_mavsim.message_types.msg_plane import MsgPlane
from rrt_mavsim.tools.plane_projections import map_3D_to_2D_planeMsg
from eVTOL_BSplines.path_generation_helpers.staticFlightPath import staticFlightPath
import cvxpy as cp
from tools.rotations import theta_to_rotation_2D

class takeoffOptimizer:


    def __init__(self,
                 plane: MsgPlane,
                 d: int = 3,
                 M: int = 10,
                 numDimensions: int = 2,
                 numPointsVelBounds: int = 8):
        
        self.plane = plane
        
        #creates the static fligth path object
        self.staticFlightPath = staticFlightPath()
        
        self.d = d
        self.M = M

        self.numDimensions = numDimensions
        self.numSidesVelBounds = numPointsVelBounds



        #sets the offset for the partition
        self.partitionOffset = 2
        #creates the mixing matrix to mix together the first and second position control points
        #to get an equivalent velocity control point
        self.mixingMatrix = np.concatenate((
            -np.eye(numDimensions), np.eye(numDimensions)
        ), axis=1)

        potato = 0


    def getOptimizedPoints(self,
                           startConditions_3D: list[np.ndarray],
                           endVelocity: float,
                           endAltitude: float,
                           numControlPoints: int):
        startConditions_2D = []
        for condition_3D in startConditions_3D:
            condition_2D = map_3D_to_2D_planeMsg(vec_3D=condition_3D,
                                                 plane_msg=self.plane)
            startConditions_2D.append(condition_2D)

        #gets the initial and final control points
        startControlPoints = self.staticFlightPath.getLocalizedControlPoints(conditions=startConditions_2D,
                                                                             d=self.d,
                                                                             M=self.M)
        


        #creates the unshiftedEndConditions_2D
        endConditions_2D_unshifted = [np.array([[0.0],[endAltitude]]),
                                      np.array([[endVelocity],[0.0]]),
                                      np.array([[0.0],[0.0]])]
        
        endControlPoints_unshifted = self.staticFlightPath.getLocalizedControlPoints(conditions=endConditions_2D_unshifted,
                                                                                     d=self.d,
                                                                                     M=self.M)
        
        #from these start and end control points, we can get the start and end points
        #for the variable control points for this thing.
        finalStartPoint = startControlPoints[:,-1:]

        #we know that the initial velocity will be 1 meter per second upwards, and 0 forwards,
        #the final velocity will be 25 forwards, and 0 upwards.

    #creates the fucntion to generate the boundaries on the velocity control points for the progression
    def getVelocityBounds(self,
                          numControlPoints: int,
                          numDimensions: int,
                          startVelocity: float,
                          endVelocity: float):
        
        #gets the change in velocity
        deltaVelocity = endVelocity - startVelocity
        #gets the increment of velocity
        incrementVelocity = deltaVelocity / numControlPoints


        velocityMagnitudes = []
        for i in range(numControlPoints):

            tempVelocity = startVelocity + i*incrementVelocity
            velocityMagnitudes.append(tempVelocity)


        numVariables = int(numDimensions*numControlPoints)
        #creates the cvxpy variable for the parameterization
        controlPoints_cpVar = cp.Variable((numVariables, 1))

        #the number of velocity control points is the number of control points minus one
        numVelocityControlPoints = numControlPoints - 1

        #creates the 
        
        potato = 0

    #creates the function to generate the minimum time path
    #numVariableControlPoints: the number of control points that we allow to vary 
    #(beyond the start and end control points which are fixed)
    def getMinTimePoints(self,
                         startConditions_3D: list[np.ndarray],
                         endConditions_3D: list[np.ndarray],
                         numVariableControlPoints: int,
                         objectiveFunction):
        
        startConditions_2D =conditions_3D_to_2D(conditions_3D=startConditions_3D,
                                                plane=self.plane)
        
        endConditions_2D = conditions_3D_to_2D(conditions_3D=endConditions_3D,
                                               plane=self.plane)
        
        #gets the velocities
        startVelocity = startConditions_2D[1]

        endVelocity = endConditions_2D[1]

        startVelocity_abs = np.linalg.norm(startVelocity)
        endVelocity_abs = np.linalg.norm(endVelocity)
        
        startControlPoints = self.staticFlightPath.getLocalizedControlPoints(conditions=startConditions_2D,
                                                                             d=self.d,
                                                                             M=self.M)
        
        endControlPoints = self.staticFlightPath.getLocalizedControlPoints(conditions=endConditions_2D,
                                                                             d=self.d,
                                                                             M=self.M)
        
        #gets the lengths of the start and end control points
        numStartControlPoints = (np.shape(startControlPoints))[1]
        numEndControlPoints = (np.shape(endControlPoints))[1]
        
        cpVariableLength = int(self.numDimensions*(numVariableControlPoints + numStartControlPoints + numEndControlPoints))

        #creates the cvxpy variable
        cpVariable = cp.Variable((cpVariableLength, 1))

        #creates the velocity constraints
        velocityConstraintMatrices = self.getVelocityConstraintMatrices_minTime(startVelocity=startVelocity_abs,
                                                                  endVelocity=endVelocity_abs,
                                                                  numControlPoints=numVariableControlPoints)

        #creates the list of constraints on the control point
        self.getVelocityConstraints(controlPoints_var=cpVariable,
                                    numDimensions=self.numDimensions,
                                    velocityConstraintMatrices=velocityConstraintMatrices)

        potato = 0


    #creates the 

    #creates the velocity constraints for the minimum time control points
    #just the floats for the min and max velocities for each control point
    def getVelocityConstraintMatrices_minTime(self,
                                       startVelocity: float,
                                       endVelocity: float,
                                       numControlPoints: int):

        #creates the number of velocity control points
        numControlPoints_velocity = numControlPoints - 1
        
        delta_velocity = endVelocity - startVelocity
        incremental_velocity = delta_velocity / numControlPoints_velocity

        #creates the list of the a and b matrices
        linearConstraintMatrices = []

        maxVelocities = []
        for i in range(numControlPoints_velocity):
            #gets the maximum velocity (from which to create the constraints)
            maxVelocity = startVelocity + i*incremental_velocity
            maxVelocities.append(maxVelocity)

            #gets the A and b vectors
            A, b = self.getLinearConstraints(maxVelocity=maxVelocity,
                                             numOuterSides=self.numSidesVelBounds)
            
            linearConstraintMatrices.append([A, b])

        #returns the A and b matrices
        return linearConstraintMatrices

    #creates the actual constraints on the control points add adds that to the list of the 
    #constraints for the control points
    def getVelocityConstraints(self,
                               numStartControlPoints: int,
                               numEndControlPoints: int,
                               numVarControlPoints: int,
                               controlPoints_var: cp.Variable,
                               numDimensions: int,
                               velocityConstraintMatrices: list[list[np.ndarray]]):

        numControlPoints = int(controlPoints_var.size / numDimensions)

        numVelocityControlPoints = numControlPoints - 1

        controlPoint_list = []

        for i in range(numControlPoints):
            currentControlPoint = controlPoints_var[(i*numDimensions):((i+1)*numDimensions),0:1]
            controlPoint_list.append(currentControlPoint)
        
        controlPointsInequalities_list = []

        #iterates over the vleoicyt constraint matrices
        for i, A_b in enumerate(velocityConstraintMatrices):
            
            A = A_b[0]
            b = A_b[1]

            startPoint = int(i*numDimensions)
            endPoint = int((i+2)*numDimensions)

            #gets the partition of the control points in question (C(s), C(s+1))
            controlPoints_partition = controlPoints_var[startPoint:endPoint, 0:1]

            #gets the inequality constraint
            inequality_Constraint_temp = [A @ self.mixingMatrix @ controlPoints_partition <= b]

            controlPointsInequalities_list.append(inequality_Constraint_temp)
            potato = 0
        
        numControlPoints = 0
        #gets the list of the control points variables
        


    #constructs the function to obtain the linear constraints 
    def getLinearConstraints(self,
                             maxVelocity: float,
                             numOuterSides: int = 8):
        
        #creates the A and b as a list
        A_list = []
        b_list = []
        
        #gets the rotation matrix
        R_normalize = theta_to_rotation_2D(theta=-np.pi/2.0)
        
        #gets the outer points list
        outerPoints_list = self.getOuterPoints(maxVelocity=maxVelocity,
                                               numOuterSides=numOuterSides)
        numOuterPoints = len(outerPoints_list)
        for i in range(numOuterPoints):

            #gets the start point
            startPoint = outerPoints_list[i%numOuterPoints]
            #end point
            endPoint = outerPoints_list[(i+1)%numOuterPoints]

            #gets the normal point
            pointToPoint_normal = self.getPointNormal(point_start=startPoint,
                                                      point_end=endPoint)
            
            #gets the n_hat vector rotated to the right
            n_hat = R_normalize @ pointToPoint_normal
            #gets the b term
            b_term = n_hat.T @ startPoint

            A_list.append(n_hat.T)
            b_list.append(b_term)

        #creates the A and B matrices
        A = np.concatenate((A_list), axis=0)
        b = np.concatenate((b_list), axis=0)
        
        return A, b


    def getOuterPoints(self,
                       maxVelocity: float,
                       numOuterSides: int):
        
        #sets the end angle
        endAngle = np.pi/2.0
        
        #initializes the outer Point
        outerPoints = []
        for i in range(numOuterSides + 1):
            
            #gets the outer point (swings from north axis to the altitude axis)
            currentAngle = endAngle * i / (numOuterSides)

            #gets the current altitude and north
            currentNorth = maxVelocity * np.cos(currentAngle)
            currentAltitude = maxVelocity * np.sin(currentAngle)

            currentPoint = np.array([[currentNorth], [currentAltitude]])

            outerPoints.append(currentPoint)



        return outerPoints

    #gets the line between two points
    def getPointNormal(self,
                             point_start: np.ndarray,
                             point_end: np.ndarray):
        
        #gets the vector between them from start to end
        vec_interPoint = point_end - point_start
        #normalizes it
        vec_interPoint_norm = vec_interPoint / np.linalg.norm(vec_interPoint)

        #NOTE: Important, because we are going in a clockwise direction, we need to rotate the vector norm to the right side

        return vec_interPoint_norm


#possibly an old and outdated optimizer
class takeoffOptimizerMagnitude:

    def __init__(self,
                 desiredAltitude: float,
                 ):

        self.desiredAltitude = desiredAltitude



    def getLeastMagntiude(self,
                          pointsList_2D: list[np.ndarray]):
        

        combinatorialList = self.getCombinatorialList(pointsList_2D=pointsList_2D)
        vectorList_pointToPoint = self.getVectorList(combinatorialList=combinatorialList)
        
        projectedVector_list = []
        nearestPoint_list = []
        nearestPointMagnitude_list = []
        #gets the projected vectors
        for combination, vector in zip(combinatorialList, vectorList_pointToPoint):
            
            startPoint = combination[0]
            endPoint = combination[1]

            #gets the projected vector
            scalingFactor, projectedVector = self.getProjectedVector(interPointVector=vector,
                                                      startPoint=startPoint)
            
            #creates the nearest point, which is on the line, but not necessarily on the line segment
            nearestPoint_line = startPoint + projectedVector
            nearestPoint_line_magnitude = np.linalg.norm(nearestPoint_line)
            

            #gets the magnitude of the starting and ending positions
            startPoint_magnitude = np.linalg.norm(startPoint)
            endPoint_magnitude = np.linalg.norm(endPoint)
            #gets the start to end vector magnitude
            startToEnd_magnitude = np.linalg.norm(vector)

            #case the scaling factor is less than zero, in which case the nearest point on the line segment
            #is the startPoint
            if scalingFactor < 0:
                nearestPoint_segment = startPoint
                nearestPoint_segment_magnitude = startPoint_magnitude

            #case scaling factor is greater than or equal to zero and less than 
            #the start to end magnitude
            elif scalingFactor >= 0 and scalingFactor < startToEnd_magnitude:
                #then the nearest  point in the line segment is that nearest point to the line
                nearestPoint_segment = nearestPoint_line
                nearestPoint_segment_magnitude = nearestPoint_line_magnitude

            #case the scaling factor is greater than or equal to the start to end magnitude
            elif scalingFactor >= startToEnd_magnitude:
                #then the nearest point is the end point
                nearestPoint_segment = endPoint
                nearestPoint_segment_magnitude = endPoint_magnitude
            
            #appends to the list
            nearestPoint_list.append(nearestPoint_segment)
            nearestPointMagnitude_list.append(nearestPoint_segment_magnitude)
            
            projectedVector_list.append(projectedVector)



        potato = 0


    def getCombinatorialList(self,
                             pointsList_2D: list[np.ndarray]):
        

        combinatorialList = []
        #gets all of the combinatorial points
        for i in range(len(pointsList_2D) - 1):
            
            #gets the starting point
            startPoint = pointsList_2D[i]

            #iterates over the next sublist
            sublist = pointsList_2D[(i+1):]
            for j in range(len(sublist)):

                endPoint = sublist[j]
                #appends to the combinatorial list
                combinatorialList.append([startPoint, endPoint])
        
        return combinatorialList
    
    #gets the vector from start to end points of the combinatorial list
    def getVectorList(self,
                      combinatorialList: list[list[np.ndarray]]):
        

        vectorList = []

        for combination in combinatorialList:

            vector = combination[1] - combination[0]

            vectorList.append(vector)

        return vectorList
    
    
    def getProjectedVector(self,
                           interPointVector: np.ndarray,
                           startPoint: np.ndarray):
        
        #the negative vector of the start point is the vector we will be projecting
        #onto the interPointVector subspace
        startPoint_negative = -startPoint

        #gets the scaling factor
        scalingFactor = (startPoint_negative.T @ interPointVector).item(0) / (np.linalg.norm(interPointVector)**2)

        #gets the projectedVector
        projectedVector = scalingFactor*interPointVector

        return scalingFactor, projectedVector




def conditions_3D_to_2D(conditions_3D: list[np.ndarray],
                        plane: MsgPlane):
    
    conditions_2D = []
    for condition in conditions_3D:
        condition_2D = map_3D_to_2D_planeMsg(vec_3D=condition,
                                             plane_msg=plane)
        conditions_2D.append(condition_2D)
    
    return conditions_2D

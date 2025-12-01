#creates the generator which creates the conditions necessary for takeoff
import numpy as np
from rrt_mavsim.message_types.msg_plane import MsgPlane
from rrt_mavsim.tools.plane_projections import map_3D_to_2D_planeMsg
from enum import Enum


class pathTypes(str, Enum):
    PARABOLA = 'Parabola'
    


class takeoffGenerator:

    def __init__(self,
                 plane: MsgPlane):
        self.plane = plane
        pass

    #the start and end positions are given in coordinates (In 3D)
    #but the velocities are given in magnitude for easier tunin
    def generateParabolicPath(self,
                              numTransitions: int,
                              startPosition_3D: np.ndarray,
                              endPosition_3D: np.ndarray,
                              startVelocity: float,
                              endVelocity: float,
                              startAccel: float,
                              endAccel: float):
        
        #gets the positions in 2D
        startPosition_2D = map_3D_to_2D_planeMsg(vec_3D=startPosition_3D,
                                                 plane_msg=self.plane)
        endPosition_2D = map_3D_to_2D_planeMsg(vec_3D=endPosition_3D,
                                               plane_msg=self.plane)
        

        #gets the start altitude and north position
        N_0 = startPosition_2D.item(0)
        A_0 = startPosition_2D.item(1)

        #the end altitude and north positions
        N_1 = endPosition_2D.item(0)
        A_1 = endPosition_2D.item(1)

        #gets the scaling factor C
        c = (N_1 - N_0) / ((A_1 - A_0)**2)

        #gets the change in altitude and north position
        delta_north = N_1 - N_0
        delta_altitude = A_1 - A_0

        #gets the changes in north
        transitionSize = delta_north / numTransitions

        position_list = []
        position_dot_list = []
        position_ddot_list = []


        #iterates over all of the transition points from start to finish
        for i in range(numTransitions + 1):
            
            current_time = transitionSize * i
            #gets the points and derivatives of the prarabola
            position, position_dot, position_ddot =\
                  self.getPointParabola(t=current_time,
                                        alpha=1.0,
                                        c=c,
                                        vertex_2D=startPosition_2D)
            position_list.append(position)
            position_dot_list.append(position_dot)
            position_ddot_list.append(position_ddot)


        potato = 0




    #defines the function to get the individual points for a specific time
    def getPointParabola(self,
                         t: float,
                         alpha: float,
                         c: float,
                         vertex_2D: np.ndarray):
        
        #gets the start altitude and north position
        N_0 = vertex_2D.item(0)
        A_0 = vertex_2D.item(1)

        #gets the north position as a function of time
        north = alpha*t

        #gets the altitude as a function of time
        altitude = np.sqrt((alpha*t - N_0)/c) + A_0

        #creates the array for the position
        output_position_2D = np.array([[north],
                                       [altitude]])
        
        '''
        #creates the slope for each of these
        north_dot = alpha
        altitude_dot = (alpha/(2*c))*((alpha/c)*t - N_0/c)**(-1/2)
        output_position_dot_2D = np.array([[north_dot],
                                           [altitude_dot]])


        #second derivative
        north_ddot = 0.0
        altitude_ddot = (-alpha**2/(4*c**2))*((alpha/c)*t - N_0/c)
        output_position_ddot_2D = np.array([[north_ddot],
                                            [altitude_ddot]])
        #'''

        
        return output_position_2D, 0, 0#, output_position_dot_2D, output_position_ddot_2D
    



    def generatePath()
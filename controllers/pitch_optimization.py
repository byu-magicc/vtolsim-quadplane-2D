#implements the class to calculate and implement the pitch optimization piece
from typing import overload
from scipy.optimize import minimize
from message_types.msg_state import MsgState
from message_types.msg_trajectory import MsgTrajectory
from controllers.forceCalculator import forceCalculator

import parameters.pitchOptimizationParameters as PITCH
import parameters.anaconda_parameters as CONDA
from tools.rotations import theta_to_rotation_2D
from tools.gamma import getGamma
import numpy as np

from typing import overload, Literal, Tuple
import numpy as np
import numpy.typing as npt

class PitchOptimization:

    def __init__(self,
                 state: MsgState,
                 Ts: float,
                 scaling: float = CONDA.pitchOptScaling,
                 p_norm_order: int = 1):

        #creates an instance of the force calculator
        self.forceCalculator = forceCalculator()

        #saves the order of the p
        self.p_norm_order = p_norm_order

        self.Ts = Ts

        self.scaling = scaling

        #creates the previous theta sample. initializes it to the state's theta
        self.theta_prev = np.array([state.theta])

        #sets the minimium value to take into account a vector
        self.minValue = 0.001

        self.gamma_ref_list = []
        self.constraints_list = []
        self.theta_list = []

        self.objectiveList = []
        self.ForcesDifferenceList = []

    #Arguments:
    #state: the current state of the aircraft
    #state_ref: the desired trajectory of the aircraft
    #F_des_i: the desired force in the inertial frame
    def update(self,
               state: MsgState,
               state_ref: MsgTrajectory,
               F_des_i: np.ndarray):
        
        #gets the flight path angle gamma
        gamma_ref = getGamma(state_ref=state_ref)

        #gets the theta constraints
        theta_constraints = self.createConstraints(gamma=gamma_ref,
                                                   theta_prev_array=self.theta_prev,
                                                   Ts=self.Ts)
        

        #creates the arguments for the args
        objective_args = (F_des_i,
                          state,
                          gamma_ref)
        
        #given the constraints on theta and so forth, finds the optimized theta
        theta_result = minimize(fun=self.objectiveFunction,
                                x0=self.theta_prev,
                                args=objective_args,
                                bounds=theta_constraints,
                                options={'maxiter': PITCH.max_iter})

        #gets the theta from the theta result
        theta = (theta_result.x).item(0)

        #for reference, we obtain the value of the objective function for each timestep for analysis
        objectiveReturns = self.objectiveFunction(theta_array_variable=theta_result.x,
                                                F_des_i=F_des_i,
                                                state=state,
                                                gamma=gamma_ref,
                                                returnExtra=True)

        self.gamma_ref_list.append(gamma_ref)
        self.theta_list.append(theta)
        self.constraints_list.append(theta_constraints)
        objectiveTemp = objectiveReturns[0]
        ForcesDifference = objectiveReturns[1]
        self.objectiveList.append(objectiveTemp)
        self.ForcesDifferenceList.append(ForcesDifference)

        #saves the current theta to the previous theta
        self.theta_prev = np.array([theta])

        #returns the theta item 0
        return theta


    #defines the overloads for the objective function
    #so I can get more variables back in an alternate definition if I so desire.
    @overload
    def objectiveFunction(self,
                          theta_array_variable: np.ndarray,
                          F_des_i: np.ndarray,
                          state: MsgState,
                          gamma: float,
                          returnExtra: Literal[False]) -> float: ...

    @overload
    def objectiveFunction(self,
                          theta_array_variable: np.ndarray,
                          F_des_i: np.ndarray,
                          state: MsgState,
                          gamma: float,
                          returnExtra: Literal[True])-> Tuple[float, np.ndarray]: ...


    #creates the optimization's objective function
    def objectiveFunction(self,
                          theta_array_variable: np.ndarray,
                          F_des_i: np.ndarray,
                          state: MsgState,
                          gamma: float,
                          returnExtra: bool = False):
        
        #gets the theta from the theta as an array
        #it is called the theta variable, because it is what we are modifying
        #to find the optimal theta to minimize the cost equation
        theta_variable = theta_array_variable.item(0)

        R_i2b = theta_to_rotation_2D(theta=theta_variable)
        R_b2i = R_i2b.T

        #gets the forces desired in the body frame
        F_des_b =  R_b2i @ F_des_i

        #alpha = theta - gamma
        #gets the resultant alpha variable given the theta variable and gamma
        alpha = theta_variable - gamma
        
        #gets the Va from the state message
        Va = state.Va

        forcesMomentsUncontrolled_body = self.forceCalculator.forces_moments_uncontrolledAero(Va=Va,
                                                                                 alpha=alpha)
        
        forcesUncontrolled_body = forcesMomentsUncontrolled_body[0:2,:]
        
        #gets the forces difference (the difference between what force we want, and what we can achieve)
        forcesDifference = F_des_b - forcesUncontrolled_body

        #gets the objective
        objective = float(np.linalg.norm(forcesDifference, 
                                   ord=self.p_norm_order))

        #if we want to return the forces difference and the objective function,
        #outside of the minimization function, this applies
        if returnExtra:
            return objective, forcesDifference
        return objective


    #creates the function to get the list of constraints on theta
    def createConstraints(self,
                          gamma: float,
                          theta_prev_array: np.ndarray,
                          Ts: float):
        

        #because scipy only allows one set of bounds for each variable,
        #we have to mind the minimum max bound and the maximum min bound
        


        #gets the upper and lower bounds with respect to total theta
        theta_lower = PITCH.theta_min
        theta_upper = PITCH.theta_max


        #sets the boundary
        boundaryChange = Ts*PITCH.q_max

        #gets the upper and lower bounds with respect to theta previous 
        previous_lower = theta_prev_array.item(0) - boundaryChange
        previous_upper = theta_prev_array.item(0) + boundaryChange

        #TODO uncomment this and bring back original
        #'''
        #gets the maximum of the min bounds
        minBound = max(theta_lower, previous_lower)
        #gets the minimum of the max bounds
        maxBound = min(theta_upper, previous_upper)
        #'''


        #creates the bounds for the device
        thetaBounds = [(minBound, maxBound)]

        return thetaBounds


    def getLists(self):
        return self.gamma_ref_list, self.constraints_list, self.theta_list

    def getObjectiveLists(self):
        return self.objectiveList, self.ForcesDifferenceList




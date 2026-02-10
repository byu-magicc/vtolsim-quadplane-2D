#implements the class to calculate and implement the pitch optimization piece

from scipy.optimize import minimize
from message_types.msg_state import MsgState
from message_types.msg_trajectory import MsgTrajectory
from controllers.forceCalculator import forceCalculator

import parameters.pitchOptimizationParameters as PITCH
from tools.old.rotations import theta_to_rotation_2d
from tools.gamma import getGamma
import numpy as np

class PitchOptimization:

    def __init__(self,
                 state: MsgState,
                 Ts: float,
                 p_norm_order: int = 1):

        #creates an instance of the force calculator
        self.forceCalculator = forceCalculator()

        #saves the order of the p
        self.p_norm_order = p_norm_order

        self.Ts = Ts

        #creates the previous theta sample. initializes it to the state's theta
        self.theta_prev = np.array([state.theta])

        #sets the minimium value to take into account a vector
        self.minValue = 0.001

    #Arguments:
    #state: the current state of the aircraft
    #state_ref: the desired trajectory of the aircraft
    #F_des_i: the desired force in the inertial frame
    def update(self,
               state: MsgState,
               state_ref: MsgTrajectory,
               F_des_i: np.ndarray):
        
        #gets the flight path angle gamma
        gamma = getGamma(state_ref=state_ref)

        #gets the theta constraints
        theta_constraints = self.createConstraints(gamma=gamma,
                                                   theta_prev_array=self.theta_prev,
                                                   Ts=self.Ts)
        

        #creates the arguments for the args
        objective_args = (F_des_i,
                          state,
                          gamma)
        
        #given the constraints on theta and so forth, finds the optimized theta
        theta_result = minimize(fun=self.objectiveFunction,
                                x0=self.theta_prev,
                                args=objective_args,
                                bounds=theta_constraints,
                                options={'maxiter': PITCH.max_iter})

        #gets the theta from the theta result
        theta = (theta_result.x).item(0)

        #returns the theta item 0
        return theta
   


    #creates the optimization's objective function
    def objectiveFunction(self,
                          theta_array: np.ndarray,
                          F_des_i: np.ndarray,
                          state: MsgState,
                          gamma: float):
        
        #gets the theta from the theta as an array
        theta = theta_array.item(0)

        Rot = theta_to_rotation_2d(theta=theta)
        #gets the forces desired in the body frame
        F_des_b =  Rot @ F_des_i



        #temp to get the aerodynamic forces
        '''
        aerodynamicForces = self.forceCalculator.getAerodynamicForces(currentState=state,
                                                                      theta=0.0,
                                                                      gamma=gamma,
                                                                      Va=25.0)
        '''
        aerodynamicForces = self.forceCalculator.forces_moments_nonProp(delta=)
        
        #gets the forces difference (the difference between what force we want, and what we can achieve)
        forcesDifference = F_des_b - aerodynamicForces

        #gets the objective
        objective = np.linalg.norm(forcesDifference, 
                                   ord=self.p_norm_order)
        

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

        #gets the upper and lower bounds with respect to theta previous 
        previous_lower = theta_prev_array.item(0) - Ts*PITCH.q_max
        previous_upper = theta_prev_array.item(0) + Ts*PITCH.q_max

        #gets the maximum of the min bounds
        minBound = max(theta_lower, previous_lower)
        #gets the minimum of the max bounds
        maxBound = min(theta_upper, previous_upper)


        #creates the bounds for the device
        thetaBounds = [(minBound, maxBound)]

        return thetaBounds

        





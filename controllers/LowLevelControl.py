import os, sys

from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))


import numpy as np
from controllers.feedforwardControl import feedForwardControl
from message_types.msg_delta import MsgDelta
from message_types.msg_state import MsgState
import parameters.anaconda_parameters as CONDA
from tools.rotations import *

import scipy.optimize as spo
from copy import copy
from scipy.optimize import minimize

from controllers.wrenchCalculation import wrenchCalculator
from models.quadplane_dynamics import QuadplaneDynamics
import parameters.simulation_parameters as SIM
import parameters.control_allocation_parameters as CAP




#creates the low level control class
class LowLevelControl:

    #creates the init function
    def __init__(self,
                 Ts: float = SIM.ts_simulation,
                 directTorqueControl: bool = False):
        

        #saves the time seperation
        self.Ts = Ts
        #saves the bool to set whether we are performing direct torque control or controlling 
        #using a desired omega
        self.directTorqueControl = directTorqueControl

        #creates an instance of the wrench calculator class
        self.wrenchCalculator = wrenchCalculator()

        self.state = MsgState()

        self.delta_prev = np.array([0.0, 0.0, 0.0, 0.0])

        self.state = MsgState()

    #creates the update function
    #Arguments:
    #1. F_des_i: the desired force input to the system, in the inertial frame
    #2. state: the current state of the system
    def update(self,
               F_des_i: np.ndarray = np.array([[0.0], [0.0]]),
               state: MsgState = MsgState()):

        self.state = state

        #gets the desired force in the body frame

        return 0
    


    def computeOptimization(self,
                            wrenchDesired: np.ndarray):
        
        #sets the x0 initial guess for the optimization function
        x0_delta = self.delta_prev

        #obtains the delta result for the delta result for the minimization function
        delta_result = minimize(fun=self.objectiveFunctionGradient,
                                x0=x0_delta,
                                args=(wrenchDesired),
                                bounds=CAP.actuatorBounds,
                                jac=True,
                                options={'maxiter': CAP.max_iter})
        
        #gets the delta array  from the result
        deltaArray = delta_result.x

        #saves the previous solution
        self.delta_prev = deltaArray

        #converts to the output delta message
        deltaOutput = MsgDelta()
        deltaOutput.from_array(deltaArray)

        return deltaOutput

    #creates the objective function for the optimizer without the gradient
    def objectiveFunction(self, deltaArray: np.ndarray, wrenchDesired: np.ndarray):
        #converts from the deltaArray to a delta Message
        deltaMessage = MsgDelta()
        deltaMessage.from_array(delta_array=deltaArray)

        #saves the mixing matrix for the wrenches
        K_Wrench = CAP.K_Wrench

        #gets the wrench actual and the wrench actual Jacobian
        wrenchActual, wrenchActualJacobian = self.wrenchCalculator.forces_moments_derivatives(delta=deltaMessage,
                                                                                              state=self.state)
        
        #gets the wrench error
        wrenchError = wrenchActual - wrenchDesired

        #creates the objective
        objective = 0.5*wrenchError.T @ K_Wrench @ wrenchError


        #saves the actual wrench
        self.wrenchActual = wrenchActual

        #returns the objective and the objective gradient
        return objective[0]


    #creates the objective function for optimizer with the gradient
    def objectiveFunctionGradient(self, 
                                  deltaArray: np.ndarray, 
                                  wrenchDesired: np.ndarray):

        #converts from the deltaArray to a delta Message
        deltaMessage = MsgDelta()
        deltaMessage.from_array(delta_array=deltaArray)

        #saves the mixing matrix for the wrenches
        K_Wrench = CAP.K_Wrench

        #gets the wrench actual and the wrench actual Jacobian
        wrenchActual, wrenchActualJacobian = self.wrenchCalculator.forces_moments_derivatives(delta=deltaMessage,
                                                                                              state=self.state)
        
        #gets the wrench error
        wrenchError = wrenchActual - wrenchDesired

        #creates the objective
        objective = 0.5*wrenchError.T @ K_Wrench @ wrenchError


        #saves the actual wrench
        self.wrenchActual = wrenchActual


        #creates the objective gradient
        objective_gradient = wrenchActualJacobian @ K_Wrench @ wrenchError


        #returns the objective and the objective gradient
        return objective, objective_gradient
    

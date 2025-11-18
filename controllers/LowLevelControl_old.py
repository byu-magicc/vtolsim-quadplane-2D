#This file implements the force and torque tracking components which we desire.


import os, sys

from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))


import numpy as np

#imports the delta message type
from message_types.msg_delta import MsgDelta
#imports the state message
from message_types.msg_state import MsgState

import parameters.anaconda_parameters as QP

from controllers.p_control import PControl


from tools.rotations import *

import scipy.optimize as spo
from copy import copy
from scipy.optimize import minimize

from controllers.wrenchCalculation import wrenchCalculator

from models.quadplane_dynamics import QuadplaneDynamics

import parameters.simulation_parameters as SIM

import parameters.control_allocation_parameters as CAP

import scipy.optimize as spo



#creates the low level control class
class LowLevelControl:


    #creates the init function
    def __init__(self, 
                 ts: float = SIM.ts_simulation,
                 torqueControl: bool = False):

        #creates an instance of the wrench calculator class
        self.wrenchCalculator = wrenchCalculator()

        #creates an instance of the state message class
        self.state = MsgState()

        #creates the previous delta solution
        self.delta_previous = np.array([0.0, 0.0, 0.0, 0.0])
        
        # control gains: q-channel
        q_kp = 0.08

        #maximum torque control
        tau_max = 5.0
        
        #creates the proportional control to go from q desired to tau desired
        self.q_ctrl = PControl(kp=q_kp,
                               limit=tau_max)
        
        #saves the torque control vector
        self.torqueControl = torqueControl

        #saves the state
        self.state = MsgState()


        #creates the array to store the actual wrench
        self.wrenchActual = np.ndarray((3,1))
        

    #creates the update function
    def update(self,
               f_d: np.ndarray = np.array([[0.0], [0.0]]), #The desired 2x1 force vector
               state: MsgState = MsgState(), #the true state of the quadplane,TODO add wind portion
               tau_desired: np.ndarray = np.array([[0.0]]), #The desired 1x1 torque vector
               omega_d: np.ndarray = np.array([[0.0]])): #The desired 1x1 omega (q) variable vector


        #saves the state
        self.state = state

        #case we use the torque control
        if self.torqueControl:
            #if this is the case, we save the tau_desired
            tau_d = tau_desired
        #case, we are not using the torque control, and we are using omega control
        else:
            #updates the q control
            tau_d = np.array([[self.q_ctrl.update(y_ref=omega_d.item(0), y=state.omega.item(0))]])

        
        #gets the whole wrench desired
        wrenchDesired = np.concatenate((f_d, omega_d), axis=0)

        #gets the delta solution
        delta = self.computeOptimization(wrenchDesired=wrenchDesired)

        #returns the delta
        return delta


    

    #creates the compute optimization function here
    def computeOptimization(self, wrenchDesired: np.ndarray):

        #sets the x0 initial guess for the optimization function
        x0_delta = self.delta_previous


        #obtains the delta result for the delta result for the minimization function
        delta_result = minimize(fun=self.objectiveFunctionGradient,
                                x0=x0_delta,
                                args=(wrenchDesired),
                                bounds=CAP.actuatorBounds,
                                jac=True,
                                options={'maxiter': CAP.max_iter})
        
        #gets the delta array from the result
        deltaArray = delta_result.x

        #saves the previous solution
        self.delta_previous = deltaArray


        #converts to the output delta message
        deltaOutput = MsgDelta()
        deltaOutput.from_array(deltaArray)

        ##########################################################################################
        #section to test the analytical jacobian versus the numerical jacobian
        objective, objectiveGradient = self.objectiveFunctionGradient(deltaArray=deltaArray,
                                                              wrenchDesired=wrenchDesired)
        
        #gets the scipy gradient
        scipyGradient = spo.approx_fprime(deltaArray, self.objectiveFunction, np.float64(1.4901161193847656e-08),  wrenchDesired)


        gradientError = objectiveGradient[:,0] - scipyGradient

        ##########################################################################################


        #returns the delta output array
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
    def objectiveFunctionGradient(self, deltaArray: np.ndarray, wrenchDesired: np.ndarray):

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
    




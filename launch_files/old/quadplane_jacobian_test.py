#This file is created for the purpose of testing the error between the gradients for the aircraft

import os, sys
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))
import numpy as np
np.errstate(overflow="ignore")
from copy import deepcopy
import parameters.simulation_parameters as SIM
import parameters.anaconda_parameters as CONDA
from models.old.quadplane_dynamics import QuadplaneDynamics
from tools.trim import compute_trim, compute_ss_model, print_ss_model
from controllers.autopilot_fixedWing import Autopilot
from viewers.view_manager import ViewManager
from message_types.msg_delta import MsgDelta
from message_types.msg_autopilot import MsgAutopilot_FixedWing
from message_types.msg_sensors import MsgSensors
from controllers.wrenchCalculation import wrenchCalculator
import time
from tools.old.rotations import *
import scipy.optimize as spo
import matplotlib.pyplot as plt


#import the low level controller
from controllers.LowLevelControl import LowLevelControl



#initializes the quadplane and the viewer
quadplane = QuadplaneDynamics(SIM.ts_simulation)
viewers = ViewManager(animation=True, data=True)

#instantiates the autopilot
autopilot = Autopilot(ts_control=SIM.ts_control)

#creates the wrench calculator
wrench_calculator = wrenchCalculator()


#creates the command for the autopilot
autopilot_commands = MsgAutopilot_FixedWing()

#creates the Low level controller
low_level_controller = LowLevelControl(ts=SIM.ts_simulation, torqueControl=True)

#sets the autopilot commands
autopilot_commands.airspeed_command = 20.0
autopilot_commands.altitude_command = 100.0

counter = 0


#creates the array to store the matrix norms
jacobianMatrixNorms = []

#sets the start and end time
sim_time = SIM.start_time
end_time = SIM.end_time


#iterates through all the possible times
while sim_time < end_time:

    current_wind = np.zeros((4, 1))

    #gets the delta from the commands
    delta = autopilot.update(cmd=autopilot_commands,
                             state=quadplane.true_state)
    
    #gets the force on the aircraft
    f_d = np.array([[0.0],
                    [0.0]])

    tau_d = np.array([[0.0]])

    
    #updates the quadplane based on those inputs
    quadplane.update(delta=delta, wind=current_wind)

    ###########################################################################################################
    #based on the current state, we obtain the wrench calculation
    calculatedWrench_noGravity = wrench_calculator.forces_moments_achieved(delta=delta, state=quadplane.true_state)

    #gets the rotation from the body to the inertial frame
    R_body2inertial_3d = quadplane.true_state.R
    #converts from rotation to euler
    phi, theta, psi = rotation_to_euler(R=R_body2inertial_3d)
    #gets the Rotation from body to inertial 2d case
    R_body2inertial_2d = theta_to_rotation_2d(theta=theta)
    #gets the rotation from the inertial to the body frame
    R_inertial2body_2d = R_body2inertial_2d.T



    #gets the calculated wrench with gravity
    gravity_body = R_inertial2body_2d @ np.array([[0.0],[CONDA.gravity*CONDA.mass]])
    
    #creates the calculated wrench with gravity
    calculatedWrench_withGravity = np.array([[calculatedWrench_noGravity.item(0) + gravity_body.item(0)],
                                             [calculatedWrench_noGravity.item(1) + gravity_body.item(1)],
                                             [calculatedWrench_noGravity.item(2)]])
    

    #gets the actual wrench
    actualWrench = quadplane._forces_moments(delta=delta)

    #gets the wrench error
    wrenchError = actualWrench - calculatedWrench_withGravity






    #next, we compare the jacobians for the two things
    wrenchJacobian = wrench_calculator.wrench_Jacobian(delta=delta, state=quadplane.true_state)


    #gets the delta as an array
    deltaArray = (delta.to_array())[:,0]
    #gets the jacobian
    scipyJacobian = spo.approx_fprime(deltaArray, quadplane.forces_moments_wrapper, np.float64(1.4901161193847656e-08))

    #gets the jacobian error
    jacobian_error = wrenchJacobian.T - scipyJacobian


    #gets the matrix norm of the jacobian error
    jacobianErrNorm = np.linalg.norm(jacobian_error)

    jacobianMatrixNorms.append(jacobianErrNorm)

    if counter % 100 == 0:
        papas_fritas = 0
    ###########################################################################################################


    #updates the viewers
    viewers.update(sim_time=sim_time,
                   true_state=quadplane.true_state,
                   estimated_state=quadplane.true_state,
                   commanded_state=quadplane.true_state,
                   delta=delta,
                   measurements=MsgSensors())
    

    #increments the simulation time
    sim_time += SIM.ts_simulation

    time.sleep(SIM.sleep_time)

    #increments the counter 
    counter += 1




#converts to an array
jacobianMatrixNorms = np.array(jacobianMatrixNorms)


plt.figure(0)
plt.plot(jacobianMatrixNorms)
plt.show()

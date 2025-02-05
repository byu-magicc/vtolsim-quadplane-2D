#This file implements a test for the minimization function for the other thing
import os, sys
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[2]))

import numpy as np
import parameters.anaconda_parameters as QP
from tools.rotations import *
from scipy.optimize import minimize



#creates the Force Desired objective
Force_desired_inertial = np.array([[0.0],
                                   [-QP.mass*QP.gravity]])

Va = 20.0

#defines the objective function
def objective(theta: float):

    #gets the rotation from body to inertial
    R_body2inertial = theta_to_rotation_2d(theta=theta)
    #gets  the rotation from the inertial to the body
    R_inertial2body = np.transpose(R_body2inertial)

    #gets the rotation from the wind to the inertialframe
    R_wind2body = alphaToRotation(alpha=theta)

    #gets the error vector
    errorVector = R_inertial2body @ Force_desired_inertial - \
            0.5*QP.rho*QP.S_wing*(Va**2)*\
            R_wind2body @ np.array([[QP.C_D_0 + theta*QP.C_D_alpha],
                                    [QP.C_L_0 + theta*QP.C_L_alpha]])
    
    #gets the objective
    J = np.abs(errorVector.item(0)) + np.abs(errorVector.item(1))

    #returns the objective
    return J


theta_0 = 0.0

#calls the minimization function
theta_result = minimize(objective,
                        theta_0)




theta = theta_result.x.item(0)


print(theta)
#This file implements the controller to dictate whether
#or not we will be in stall conditions. 
import os, sys
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))

import parameters.anaconda_parameters as QP
import numpy as np

#creates the function to get whether we are within stall conditions
def withinStallConditions(alpha_degrees: float)->tuple[bool, float]:
    

    alpha = np.radians(alpha_degrees)
    #creates the variable for inConditions
    inConditions = False
    #creates the variable for the error
    attackError = 0
    if alpha > QP.maximumAlpha:
        inConditions = False
        attackError = alpha - QP.maximumAlpha
    elif alpha < QP.minimumAlpha:
        inConditions = False
        attackError = alpha - QP.maximumAlpha
    else:
        inConditions = True


    #returns the two variables
    return inConditions, np.degrees(attackError)






temp, error = withinStallConditions(alpha_degrees=25.0)



potato = 0
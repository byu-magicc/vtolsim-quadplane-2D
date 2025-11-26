#implements the test for the takeoff of the aircraft
import os, sys
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))

import numpy as np

from eVTOL_BSplines.path_generation_helpers.staticFlightPath import staticFlightPath
from eVTOL_BSplines.path_generation_helpers.conditions import conditions

from controllers.highLevelControl import highLevelControl
from controllers.LowLevelControl import LowLevelControl
from controllers.feedforwardControl import feedForwardControl

from message_types.msg_delta import MsgDelta
from message_types.msg_state import MsgState

numDimensions = 2
numConditions = 3


flightConditions = conditions(dimension=numDimensions,
                              numConditions=numConditions)


#creates the init and final conditions
pos_init = np.array([[0.],[0.]])
vel_init = np.array([[0.],[0.]])
accel_init = np.array([[9.8],[0.]])


conditions_init = [pos_init, vel_init, accel_init]


pos_final = np.array([[1000.],[100.]])
vel_final = np.array([[25.],[0.]])
accel_final = np.array([[0.],[0.]])

conditions_final = [pos_final, vel_final, accel_final]

rho = np.array([1.0, 1.0, 1.0])

#creates the flight generator
flightGen = staticFlightPath()
#gets the control points from the flight generator
controlPoints = flightGen.getControlPoints(initialConditions=conditions_init,
                                           finalConditions=conditions_final,
                                           rho=rho,
                                           numDimensions=numDimensions,
                                           )



potato = 0
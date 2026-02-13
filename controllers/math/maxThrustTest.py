#This file implements the max thrust testing function
import os, sys
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[2]))
import numpy as np

from models.old.quadplane_dynamics import QuadplaneDynamics
import parameters.simulation_parameters as SIM

import matplotlib.pyplot as plt

#instantiates the quadplane dynamics file
quadplane = QuadplaneDynamics(ts=SIM.ts_simulation)




#creates a linspace for the Va
Va_space = np.linspace(-40.0, 40.0, 1000)



#creates the lists for the thrusts
informationList = []

for i in range(1000):
    currentVa = Va_space.item(i)
    thrust, torque = quadplane._motor_thrust_torque_advanced(Va=currentVa, delta_t=1.0)
    #creates the array of the thrusts
    informationVector = np.array([[currentVa],
                                    [thrust]])
    informationList.append(informationVector)



#converts to an array
informationArray = np.array(informationList)[:,:,0].T

#gets the max thrust
thrusts = informationArray[1,:]

#gets the airspeeds
airspeeds = informationArray[0,:]

maxThrust = 0.0
maxThrustIndex = 0
for i in range(1000):
    currentThrust = thrusts.item(i)
    if currentThrust > maxThrust:
        maxThrust = currentThrust
        maxThrustIndex = i

print("Max Thrust: ", maxThrust)
print("max thrust index: ", maxThrustIndex)
print("max thrust Va: ", informationArray[0,maxThrustIndex])

print("Thrust at Zero Airspeed: ", thrusts[500])



#plots the maximum thrusts
plt.figure(0)
plt.plot(airspeeds, thrusts)
plt.show()

potato = 0
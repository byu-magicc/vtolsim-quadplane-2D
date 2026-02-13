from message_types.msg_trajectory import MsgTrajectory
import numpy as np

minValue = 0.001



#gets the flight path angle gamma
def getGamma(state_ref: MsgTrajectory):
    
    vel_ref = state_ref.vel
    
    #gets the z component of the velocity
    vel_ref_x = vel_ref.item(0)
    vel_ref_z = vel_ref.item(1)
    #case there is a negligible value in the x place (due to the nonlinear nature of the arctan function)
    if np.abs(vel_ref_x) < minValue:
        vel_ref_z_sign = np.sign(vel_ref_z)
        #case vel_ref z is positive
        if vel_ref_z_sign >= 0:
            gamma = np.pi/2.0
        #case, it is negative
        else:
            gamma = -np.pi/2.0    
    else:
        #gets the flight path angle gamma
        gamma = np.arctan2(-vel_ref_z, vel_ref_x)
    return gamma
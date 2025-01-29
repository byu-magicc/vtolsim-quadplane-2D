#This file implements the autopilot for the fixed wing version of the plane.

import numpy as np
import parameters.control_parameters as AP
from controllers.pi_control import PIControl
from controllers.pid_control import PIDControl
from controllers.pd_control_with_rate import PDControlWithRate

from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta
from message_types.msg_autopilot import MsgAutopilot
from tools.saturate import saturate
from tools.rotations import *

#creates the autopilot class
class Autopilot:


    #creates the initialization function
    def __init__(self, ts_control: float):

        #creates the instantiation of the three autopilots for the three things we are tracking
        #creates the function that gets the pitch control from the altitude command
        self.pitch_from_altitude = PIControl(kp=AP.altitude_kp,
                                             ki=AP.altitude_ki,
                                             Ts=ts_control,
                                             limit=np.radians(30))#sets the pitch limit to 30 degrees
        
        #creates the function that gets the elevator control from a pitch command
        self.elevator_from_pitch = PDControlWithRate(kp=AP.pitch_kp,
                                                     kd=AP.pitch_kd,
                                                     limit=1.0)#TODO make sure this is the correct limit


        #creates the controller to get the throttle command from the airspeed command
        self.throttle_from_airspeed = PIControl(kp=AP.airspeed_throttle_kp,
                                                ki=AP.airspeed_throttle_ki,
                                                Ts=ts_control,
                                                limit=1.0)
        
        #creates the commanded state variable
        self.commanded_state = MsgState()


    #creates the update function
    def update(self,
               cmd: MsgAutopilot,
               state: MsgState):
        
        #creates the lateral autopilot

        #gets the actual altitude, which is the minus of the down position
        altitude = -state.pos.item(2)
        
        #gets the rotation matrix
        R_body2inertial = state.R
        #converts to euler
        phi, theta, psi = rotation_to_euler(R=R_body2inertial)
        #gets the q value
        q = state.omega.item(1)



        ################################################################################
        #altitude control portion
        ################################################################################
        #gets the saturated altitude command
        altitude_command_sat = saturate(cmd.altitude_command, 
                                        altitude - AP.altitude_zone, 
                                        altitude + AP.altitude_zone)
        #gets the theta control from the altitude command 
        # (Run through the outer PI loop)
        theta_command = self.pitch_from_altitude.update(y_ref=altitude_command_sat,
                                                        y=altitude)
        #gets the delta_e control from the theta command
        delta_e = self.elevator_from_pitch.update(y_ref=theta_command,
                                                  y=theta,
                                                  ydot=q)
        

        ################################################################################
        #airspeed control portion
        ################################################################################
        #updates the airspeed controller
        delta_t_thrust = self.throttle_from_airspeed.update(y_ref=cmd.airspeed_command,
                                                            y = state.Va)
        
        #creates the delta message
        delta = MsgDelta(elevator=delta_e,
                         throttle_thrust=delta_t_thrust)
        
        #returns the delta
        return delta



import numpy as np

#defines the autopilot message class
class MsgAutopilot_FixedWing:

    #creates the initialization function
    def __init__(self):
        self.airspeed_command = float(0.0)
        self.altitude_command = float(0.0)
        self.climb_rate_command = float(0.0)



#defines the autopilot for the quadrotor
class MsgAutopilot_Quadrotor:

    #
    def __init__(self):
        #creates the position command in 2d
        self.position_cmd = np.array([[0.0], #position North command
                                      [0.0], #position East command (always zero)
                                      [0.0]]) #position Down command
        #creates the velocity  command in 2d
        self.velocity_cmd = np.array([[0.0], #pos_north_dot command
                                      [0.0], #pos_east dot command (always zero)
                                      [0.0]]) #pos_down_dot command
        #creates the acceleration command
        self.accel_cmd = np.array([[0.0], #pos_north_ddot command
                                   [0.0], #pos_east ddot command (always zero)
                                   [0.0]])#pos_down_ddot command
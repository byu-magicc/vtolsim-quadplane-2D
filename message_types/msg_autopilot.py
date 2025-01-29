


#defines the autopilot message class

class MsgAutopilot:

    #creates the initializatio n function
    def __init__(self):
        self.airspeed_command = float(0.0)
        self.altitude_command = float(0.0)
        self.climb_rate_command = float(0.0)
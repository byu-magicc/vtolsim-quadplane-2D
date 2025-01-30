import numpy as np

class MsgDelta:
    '''
        Message class that defines the control inputs to the eVTOL

        Attributes:
            elevator: elevator angle in radians
            throttle_front: throttle for front rotor in [0, 1]
            throttle_rear: throttle for rear rotor in [0, 1]
            throttle_thrust:  throttle for forward thruster in [0, 1]
    '''
    def __init__(self,
                 elevator: float=0.,  
                 throttle_front: float=0.,  
                 throttle_rear: float=0.,  
                 throttle_thrust: float=0.,  

                 ):
            self.elevator = elevator  
            self.throttle_front = throttle_front   
            self.throttle_rear = throttle_rear 
            self.throttle_thrust = throttle_thrust  
    
    def from_array(self, delta_array: np.ndarray):
        self.elevator =  delta_array.item(0)
        self.throttle_front = delta_array.item(1) 
        self.throttle_rear = delta_array.item(2)
        self.throttle_thrust = delta_array.item(3)

    def to_array(self)->np.ndarray:
        '''
            Convert MsgDelta structure to array:
            :output: np.array([[
                elevator, 
                throttle_front,
                throttle_rear,
                throttle_thrust,
                ]]).T
        '''
        delta = np.zeros((4,1))
        delta[0,0] = self.elevator
        delta[1,0] = self.throttle_front
        delta[2,0] = self.throttle_rear
        delta[3,0] = self.throttle_thrust
        return delta

    def __add__(self, other):
        '''Overload the addition '+' operator'''
        out = MsgDelta()
        out.elevator = self.elevator + other.elevator
        out.throttle_front = self.throttle_front + other.throttle_front  
        out.throttle_rear = self.throttle_rear + other.throttle_rear  
        out.throttle_thrust = self.throttle_thrust + other.throttle_thrust  
        return out

    def __sub__(self, other):
        '''Overload the subtraction '-' operator'''
        out = MsgDelta()
        out.elevator = self.elevator - other.elevator
        out.throttle_front = self.throttle_front - other.throttle_front  
        out.throttle_rear = self.throttle_rear - other.throttle_rear  
        out.throttle_thrust = self.throttle_thrust - other.throttle_thrust  
        return out
    
    def __rmul__(self, other):
        '''Overload right multiply by a scalar'''
        out = MsgDelta()
        out.elevator = other * self.elevator
        out.throttle_front = other * self.throttle_front  
        out.throttle_rear = other * self.throttle_rear  
        out.throttle_thrust = other * self.throttle_thrust  
        return out


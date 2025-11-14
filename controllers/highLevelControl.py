#this is the path following algorithm we need to make the low level controller operate correctly.

from message_types.msg_state import MsgState
import numpy as np


class highLevelControl:


    def __init__(self):

        pass

    def update(self,
               position_desired: np.ndarray,
               velocity_desired: np.ndarray,
               acceleration_desired: np.ndarray,
               state: MsgState):

        

        pass
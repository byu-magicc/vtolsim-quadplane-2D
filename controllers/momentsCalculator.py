#implements the calculator to find the moment achievable 

from controllers.wrenchCalculation import wrenchCalculator
from message_types.msg_state import MsgState
import parameters.anaconda_parameters as CONDA


#we don't bother with the wrench calculator. we pull it all in again and
#calculate the different parts ourselves

class momentsCalculator:

    def __init__(self):

        pass

    #NOTE: These are not the actual aerodynamic moments
    def getAerodynamicMomentsCoefficients(self,
                              currentState: MsgState):
        
        #gets the airspeed from the current state
        Va = currentState.Va

        alpha = currentState.alpha

        #gets the q_bar 
        qbar = 0.5 * CONDA.rho * Va**2

        #gets the qbar total
        qbar_total = qbar * CONDA.S_wing * CONDA.c

        #from this gets the individual aerodynamic moments

        #the natural moment 
        M_0 = qbar_total * CONDA.C_m_0

        #the alpha Moment
        M_alpha = qbar_total*CONDA.C_m_alpha

        #the q Moment
        M_q = qbar_total*CONDA.C_m_q

        #the delta moment
        M_delta_e = qbar_total * CONDA.C_m_delta_e

        #returns all four of these
        return M_0, M_alpha, M_q, M_delta_e



#creates all of the high level controller parameters
import numpy as np
import control
import parameters.anaconda_parameters as CONDA


#creates the state transmission matrix for the error state
A = np.array([[0, 0, 1, 0],
              [0, 0, 0, 1],
              [0, 0, 0, 0],
              [0, 0, 0, 0]])

B = np.array([[0, 0],
              [0, 0],
              [1, 0],
              [0, 1]])

#creates the C output
Cr = np.array([[1,0,0,0],
               [0,1,0,0]])

#now, we need to get the augmented system for purposes of the integrator
A_1 = np.vstack((np.hstack((A, np.zeros((4,2)))),
                 np.hstack((Cr, np.zeros((2,2))))))

B_1 = np.vstack((B, np.zeros((2,2))))


####################################################################################
# Q, R sections
####################################################################################

#sets the maximum allowable desired errors
posErrorMax = 10.
posCoef = 1 / posErrorMax**2

velErrorMax = 1.
velCoef = 1 / velErrorMax**2

#sets the integrator maxes
integratorMax = 5.0
integratorCoef = 1 / integratorMax**2


#creates the Q and R matrices
Q = np.diag(np.array([posCoef, posCoef, velCoef, velCoef, integratorCoef, integratorCoef]))

R = np.array([[1, 0],
              [0, 1]])


K_t, _, _ = control.lqr(A_1, B_1, Q, R)

#gets the main K matrix and then partitions it into K and Ki parts
K = K_t[:,:4]
Ki = K_t[:,4:]




#sets the maximum derivative value for the error state. If the error derivative's value is greater than this
#then we do not integrate. This is an ANTI-WINDUP value
max_error_derivative_antiwindup = 1.0




temporaryPotato = 0
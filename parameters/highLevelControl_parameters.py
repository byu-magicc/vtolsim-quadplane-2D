#creates all of the high level controller parameters
import numpy as np
import control


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

#creates the Q and R matrices
Q = np.eye(4)
R = np.eye(2)

K, S, E = control.lqr(A, B, Q, R)

#gets the closed loop Matrix equations
Acl = A - B @ K

Acl_inv = np.linalg.inv(Acl)

#obtains the reference gain for this system, for it to have a DC gain of 1
K_ref = -np.linalg.inv(Cr @ Acl_inv @ B)

Bcl = B @ K_ref






temporaryPotato = 0
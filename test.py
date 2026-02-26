import numpy as np
from rrt_mavsim.message_types.msg_delta import MsgDelta
from scipy.optimize import linprog




A = np.array([[ 7.98408911e-01, -6.02115612e-01],
               [-6.02115612e-01, -7.98408911e-01],
               [-7.98408911e-01,  6.02115612e-01],
               [ 6.02115612e-01,  7.98408911e-01],
               [ 0.00000000e+00, -1.00000000e+00],
               [-1.00000000e+00,  0.00000000e+00],
               [ 7.10542736e-17,  0.00000000e+00],
               [ 1.00000000e+00,  0.00000000e+00],
               [-7.10542736e-17,  0.00000000e+00],
               [ 0.00000000e+00,  1.00000000e+00]])


b = np.array([[ 730.        ],
              [  75.        ],
              [  30.        ],
              [  75.        ],
              [ 216.66666667],
              [-116.66666667],
              [  25.        ],
              [ 216.66666667],
              [  25.        ],
              [-116.66666667]])

b = b.flatten()

testX = np.array([[200.0],[-200.0]])

leftSide = A @ testX

subtraction = leftSide - b



dummyVariable = np.zeros(A.shape[1])

#uses linprog to find whether or not there exists a viable solution to the problem here

result = linprog(dummyVariable, A_ub=A, b_ub=b.flatten(),bounds=[(None, None), (None, None)], method='highs')

intersectionOccurredTemp = result.success

potato = 0

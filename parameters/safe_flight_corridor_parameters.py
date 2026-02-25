import numpy as np



numDimensions = 2
rho_bspline = np.array([[1.0], [1.0], [1.0]])


degree = 3
M = 10

# sets the initial segment length
segmentLength = 700.0

#sets the number of desired initial paths
numInitPaths = 1


#sets the width and the height of the sfc
# sets the width of the SFC
sfc_width = 100.0
# sets the height of the SFC (used only for 3D)
sfc_height = 100.0

#minimum turning radius stuff
# sets the minimum turning radius
MinTurnRadius = 300.0
# using the minimum turning radius and the width, we get the maximum angle between Safe flight Corridors
# for them to be allowed to be added to
gamma_min = np.arcsin((MinTurnRadius - sfc_width) / (MinTurnRadius))
Chi_max = np.pi - 2.0 * gamma_min

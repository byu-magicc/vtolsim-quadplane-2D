#implements the obstacle avoidance algorithm
import os, sys
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))
import numpy as np
from rrt_mavsim.message_types.msg_world_map import MsgWorldMap, PlanarVTOLParams, MapTypes
from rrt_mavsim.viewers.view_manager import ViewManager
from rrt_mavsim.planners.rrt_sfc_bspline import RRT_SFC_BSpline
import rrt_mavsim.parameters.planner_parameters as PLAN
import rrt_mavsim.parameters.flightCorridor_parameters as FLIGHT_PLAN
import rrt_mavsim.parameters.planarVTOL_map_parameters as VTOL_PARAM
from viewers.view_manager import ViewManager
from rrt_mavsim.tools.smoothingTools import flight_corridors_smooth_path
from rrt_mavsim.tools.waypointsTools import getNumCntPts_list

#current file path
currentFilePath = Path(__file__).resolve()
mainDirectoryFilePath = currentFilePath.parents[1]
#sets the subfolder
lookupDirectory = mainDirectoryFilePath / 'lookupTables'

mapFileName = 'worldMap.npz'
waypointsSmoothFileName = 'waypointsSmooth.npz'
waypointsNotSmoothFileName = 'waypointsNotSmooth.npz'

mapDirectory = lookupDirectory / mapFileName
waypointsSmoothDirectory = lookupDirectory / waypointsSmoothFileName
waypointsNotSmoothDirectory = lookupDirectory / waypointsNotSmoothFileName

numDimensions = 2


mapOrigin_2D = np.array([[0.0],[0.0]])
mapOrigin_3D = np.array([[0.0],[0.0],[0.0]])
n_hat = np.array([[0.0],[1.0],[0.0]])



#creates the planar vtol params
params = PlanarVTOLParams(mapOrigin_2D=mapOrigin_2D,
                          mapOrigin_3D=mapOrigin_3D,
                          n_hat=n_hat)

startPosition = params.startPosition
endPosition = params.endPosition

worldMap = MsgWorldMap(obstacleFieldType=MapTypes.PLANAR_VTOL,
                       numDimensions_algorithm=numDimensions,
                       planarVTOL_Params=params)

viewer = ViewManager(animation=True,
                     world_map=worldMap)

planner = RRT_SFC_BSpline(numDimensions=numDimensions,
                          M=FLIGHT_PLAN.M,
                          Va=PLAN.Va0,
                          rho=FLIGHT_PLAN.rho,
                          step_length=FLIGHT_PLAN.segmentLength,
                          numDesiredInitPaths=FLIGHT_PLAN.numInitialPaths,
                          n_hat=n_hat,
                          p0=mapOrigin_3D)


planner.generatePath(startPosition_3D=startPosition,
                      endPosition_3D=endPosition,
                      worldMap=worldMap,
                      segmentLength=FLIGHT_PLAN.segmentLength)


#gets the not smooth waypoints
waypointsNotSmooth = planner.getWaypointsNotSmooth()

#gets the smooth waypoints
waypoints_smooth = flight_corridors_smooth_path(waypoints_not_smooth=waypointsNotSmooth,
                                                world_map=worldMap,
                                                angle_max=np.pi/2)

viewer.drawWaypoints(waypoints=waypoints_smooth,
                     n_hat=n_hat,
                     p0=mapOrigin_3D)




#for testing, I would like to not have to compute everything out again every time.
#I am going to save everything to an npz file



np.savez(mapDirectory, model=worldMap)
np.savez(waypointsSmoothDirectory, model=waypoints_smooth)
np.savez(waypointsNotSmoothDirectory, model=waypointsNotSmooth)


#gets the list of number of control points



potato = 0
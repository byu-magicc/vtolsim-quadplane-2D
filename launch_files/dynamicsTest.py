#implements a dynamics test
import os, sys
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))

import numpy as np

from models.quadplaneDynamics import QuadplaneDynamics

from rrt_mavsim.message_types.msg_plane import MsgPlane

import parameters.plane_parameters as PLANE
import parameters.simulation_parameters as SIM
import parameters.anaconda_parameters as CONDA

from viewers.view_manager import ViewManager
from message_types.msg_delta import MsgDelta
from message_types.msg_sensors import MsgSensors

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

#loads up those lookup tables. Unpacks it from the python pickling process
worldMapData = np.load(mapDirectory, allow_pickle=True)
waypointsSmoothData = np.load(waypointsSmoothDirectory, allow_pickle=True)
waypointsNotSmoothData = np.load(waypointsNotSmoothDirectory, allow_pickle=True)

worldMap = worldMapData['model'].item()
waypoints_smooth = waypointsSmoothData['model'].item()
waypoints_not_smooth = waypointsNotSmoothData['model'].item()

quadplane = QuadplaneDynamics(plane_msg=PLANE.plane_msg,
                              ts=SIM.ts_simulation,
                              pn0_3D=100.,
                              pd0_3D=0.,
                              pn_dot0_3D=100.,
                              pd_dot0_3D=0.,
                              theta0=np.radians(20.0),
                              q0=0.)

viewers = ViewManager(animation=True,
                      data=True,
                      world_map=worldMap,
                      msg_plane=PLANE.plane_msg)

#draws the smooth set of waypoints
viewers.drawWaypoints(waypoints=waypoints_smooth,
                     n_hat=PLANE.plane_msg.n_hat,
                     p0=PLANE.plane_msg.origin_3D,
                     color='r')

sim_time = SIM.start_time
end_time = SIM.end_time


#creates the wind as zero
wind = np.array([[0.0],[0.0],[0.0],[0.0]])

while sim_time < end_time:

    deltaInput = MsgDelta()
    quadplane.update(delta=deltaInput,
                     wind=wind)
    

    viewers.update(sim_time=sim_time,
                   true_state=quadplane.true_state,
                   estimated_state=quadplane.true_state,
                   commanded_state=quadplane.true_state,
                   delta=deltaInput,
                   measurements=MsgSensors())

    sim_time += SIM.ts_simulation

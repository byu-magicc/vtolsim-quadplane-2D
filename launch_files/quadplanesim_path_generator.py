#This file implements a testing experiment with the path_generator_simplified class
#from the eVTOL_BSplines library

from eVTOL_BSplines.path_generator_simplified import waypointPathGenerator
from eVTOL_BSplines.path_generator_simplified import Waypoint, WaypointData

import bsplinegenerator as bsg

import os, sys
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))
import numpy as np
import parameters.simulation_parameters as SIM
from models.quadplane_dynamics import QuadplaneDynamics
from message_types.msg_trajectory import MsgTrajectory
from controllers.trajectory_tracker import TrajectoryTracker
from controllers.control_allocator import ControlAllocator
from viewers.view_manager import ViewManager
from hello_package.hello import sayhello
import matplotlib.pyplot as plt

from copy import copy

#plots the path
from bsplinegenerator.bspline_plotter import plot_bspline
from eVTOL_BSplines.path_plotter import plotSpline_2d, plotSplinePositionerror_2d

import time


import pandas as pd



#creates the two waypoints
waypoint_0 = Waypoint(location=np.array([[0.0],[0.0]]),
                      velocity=np.array([[5.0],[0.0]]),
                      acceleration=np.array([[0.0],[0.0]]))




#gets the intermediate waypoints
intermediateWaypoints = np.array([[30.0, -5.0]]).T

waypoint_4 = Waypoint(location=np.array([[300.0],[-50.0]]),
                      velocity=np.array([[25.0],[-5.0]]),
                      acceleration=np.array([[0.0],[0.0]]))

#puts them together into the main waypointData
waypointData = WaypointData(start_waypoint=waypoint_0,
                            end_waypoint=waypoint_4,
                            intermediate_locations=intermediateWaypoints)

#instantiates the path generator
pathGenerator = waypointPathGenerator(waypoints=waypointData,
                                      max_curvature=1.0,
                                      degree=3,
                                      num_points_per_interval=100)


#gets the waypoint locations
waypoint_locations = pathGenerator.getWaypointLocations()



#gets the position, velocity, and accelerational data
pos_data, pos_time_data = pathGenerator.getPosData()
vel_data, vel_time_data = pathGenerator.getVelData()
accel_data, accel_time_data = pathGenerator.getAccelData()

#creates the alias of the pos_data as the bspline points
bspline_points_2d = pos_data
#adds the y axis (all zeros) to convert to 3D

#gets the shape
bspline_points_shape = np.shape(bspline_points_2d)

#gets the num of points
num_points = bspline_points_shape[1]

#bspline_points_3d 
zero_row = np.zeros((1, num_points))
#inserts that into the 2d array to get the 3d array
bspline_points_3d = np.insert(bspline_points_2d, 1, zero_row, axis=0)


#gets the control points
controlPoints = pathGenerator.getControlPoints()

#gets the knot points
knot_points = pathGenerator.getKnotPoints()

#'''
#gets the number of points
numPoints = (np.shape(pos_data))[1]



#creates the quadplane
quadplane = QuadplaneDynamics(ts=SIM.ts_simulation,
                              pn_dot0=0.0,
                              pd_dot0=0.0)

#creates the geometric tracker controller
tracker = TrajectoryTracker(ts_control=SIM.ts_simulation)

#creates the control allocator
allocator = ControlAllocator(ts_control=SIM.ts_simulation)

#instantiates the trajectory message class
trajectory = MsgTrajectory()

#creates the viewer
viewers = ViewManager(animation=True, data=True)

#calls the function to create and draw the trajectory
viewers.drawTrajectory(points=bspline_points_3d,
                       width=5.0)

#sets the sim_time
sim_time = SIM.start_time
end_time = SIM.end_time

#list to store the commanded wrenches
commandedWrenches = []
#

counter = 0

#creates the main simulation loop
print("Press CTRL-C to exit...")
while sim_time < end_time and counter < numPoints:

    #sets the estimated state
    estimated_state = quadplane.true_state

    #updates the trajectory position and velocity
    desiredPosition = pos_data[:,counter].reshape((2,1))
    desiredVel = vel_data[:,counter].reshape((2,1))
    desiredAccel = accel_data[:,counter].reshape((2,1))

    trajectory.pos = desiredPosition
    trajectory.vel = desiredVel
    trajectory.accel = desiredAccel


    # trajectory velocity, acceleration, pitch, and pitch rate all default to zero
    # -------controller-------------
    estimated_state = quadplane.true_state  # uses true states in the control
    #This is commanded wrench to be obtained from the propellers
    commanded_wrench, commanded_state = tracker.update(trajectory, estimated_state)
    delta = allocator.update(commanded_wrench, estimated_state)
    #-------update physical system-------------
    current_wind = np.zeros((4, 1))
    quadplane.update(delta, current_wind)  
    #-------update viewers-------------
    viewers.update(
        sim_time,
        quadplane.true_state,  # true states
        estimated_state,  # estimated states
        commanded_state,  # commanded states
        delta,  # inputs to aircraft
        None,  # measurements
    )


    #appends the commanded wrench
    commandedWrenches.append(commanded_wrench)

    #gets the positional error 
    actual_pos_3d = quadplane.true_state.pos
    #reduces it down to 2d
    actual_pos_2d = np.array([[actual_pos_3d.item(0)],
                              [actual_pos_3d.item(2)]])



    time.sleep(0.01)

    #increments the counter
    counter += 1
    #-------increment time-------------
    sim_time += SIM.ts_simulation


#at the end, we parse through the commanded wrenches data
commandedWrenches = np.array(commandedWrenches)[:,:,0].T

plt.figure(0)
plt.plot(commandedWrenches[0,:])
plt.title("Commanded X Force")
plt.show()

plt.figure(1)
plt.plot(commandedWrenches[1,:])
plt.title("Commanded Z Force")
plt.show()


potato = 0
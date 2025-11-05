"""
quadplane_viewer: simple quadplane viewer
        4/1/2019 - R.W. Beard
        4/15/2019 - BGM
        5/3/2019 - R.W. Beard
        1/31/2024 - RWB
"""
import pyqtgraph.opengl as gl
import pyqtgraph.Vector as Vector
from viewers.draw_quadplane import DrawQuadplane
import numpy as np
from viewers.draw_trajectory import DrawTrajectory
from time import time
from rrt_mavsim.message_types.msg_world_map import MsgWorldMap
from rrt_mavsim.viewers.draw_map import DrawMap
from rrt_mavsim.message_types.msg_waypoints import MsgWaypoints_SFC
from rrt_mavsim.viewers.draw_waypoints import DrawWaypoints



red = np.array([[204, 0, 0],
                [204, 0, 0]])/255.


purple = np.array([[170, 0, 255],
                   [170, 0, 255]])/255


class QuadplaneViewer():
    def __init__(self, app, dt = 0.01,
                 plot_period = 0.2, 
                 ts_refresh=1./30.,# time interval between a plot update
                 grid_on: bool = True,#toggles whether or not the grid is on
                 axes_on: bool = False, #toggles whether or not the x and z axes stay on with the airplane to show the orientation
                 alpha_axis_on: bool = False, #toggles whether or not the axis in the direction of velocity is on (visualizing alpha)
                 worldMap: MsgWorldMap = None):
        # initialize Qt gui application and window
        self._dt = dt
        self._time = 0
        self._plot_period = plot_period
        self._plot_delay = 0
        self.scale = 2500
        self.app = app  # initialize QT, external so that only one QT process is running
        self.window = gl.GLViewWidget()  # initialize the view object
        self.window.setWindowTitle('Quadplane Viewer')
        self.window.setGeometry(500, 0, 500, 500)  # args: upper_left_x, upper_right_y, width, height
        
        center = self.window.cameraPosition()
        center.setX(1000)
        center.setY(1000)
        center.setZ(0)
        self.window.setCameraPosition(pos=center,
                                      distance=self.scale,
                                      elevation=50,
                                      azimuth=0)
        #TODO. Make sure this actually works
        self.window.setBackgroundColor('k')
        grid = gl.GLGridItem() # make a grid to represent the ground
        grid.scale(self.scale/20, self.scale/20, self.scale/20) # set the size of the grid (distance between each line)
        
        #if the grid is on, then we add the grid as an item to the window
        if grid_on:
            self.window.addItem(grid) # add grid to viewer
        
        self.window.setCameraPosition(distance=20) # distance from center of plot to camera
        self.window.setBackgroundColor(0.25)  # set background color to black
        
        #draws the map
        DrawMap(map=worldMap,
                window=self.window)   
        
        self.window.show()  # display configured window
        self.window.raise_() # bring window to the front
        self.plot_initialized = False # has the mav been plotted yet?
        self.quadplane_plot = []
        self.ts_refresh = ts_refresh
        self.t = time()
        self.t_next = self.t  

    def drawWaypoints(self,
                      waypoints: MsgWaypoints_SFC,
                      lineColor: np.ndarray = red,
                      n_hat: np.ndarray = None,
                      p0: np.ndarray = None):
        
        DrawWaypoints(waypoints=waypoints,
                      window=self.window,
                      lineColor=lineColor,
                      n_hat=n_hat,
                      p0=p0)




    def update(self,
               state,
               ):
        # initialize the drawing the first time update() is called
        if not self.plot_initialized:
            self.quadplane_plot = DrawQuadplane(state, self.window)
            # update the center of the camera view to the quadrotor location
            # defined in ENU coordinates
            view_location = Vector(state.pos.item(1), state.pos.item(0), -state.pos.item(2))
            self.window.opts['center'] = view_location
            # redraw
            self.app.processEvents()
            self.plot_initialized = True
        # else update drawing on all other calls to update()
        else:
            t = time()
            if t-self.t_next > 0.0:
                self.quadplane_plot.update(state)
                self.t = t
                self.t_next = t + self.ts_refresh
            view_location = Vector(state.pos.item(1), state.pos.item(0), -state.pos.item(2))
            self.window.opts['center'] = view_location
            self.app.processEvents()
            self._plot_delay += self._dt

            # if self._plot_delay >= self._plot_period:
            #     self.quadplane_plot.update(state)
            #     self._plot_delay = 0
            #     # update the center of the camera view to the quadplane location
            #     # defined in ENU coordinates
            #     # redraw

    def close(self):
        self.window.close()

    def addTrajectory(self, 
                      points,
                      width: float = 2):
        #defines the color blue for the trajectory
        blue_color = np.array([[30, 144, 255, 255]])/255.
        #creates an instance of the drawtrajectory class
        self.trajectory = DrawTrajectory(points, 
                                         blue_color, 
                                         self.window,
                                         width=width)

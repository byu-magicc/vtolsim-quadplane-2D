
import pyqtgraph as pg
from viewers.quadplane_viewer import QuadplaneViewer
from viewers.data_viewer import DataViewer
from viewers.autopilot_data_viewer import AutopilotDataViewer

# from viewers.sensor_viewer import SensorViewer
import parameters.simulation_parameters as SIM
from message_types.msg_state import MsgState
from message_types.msg_sensors import MsgSensors
from message_types.msg_delta import MsgDelta
from message_types.msg_integrator import MsgIntegrator
from message_types.msg_trajectory import MsgTrajectory

from rrt_mavsim.message_types.msg_plane import MsgPlane
from rrt_mavsim.message_types.msg_world_map import MsgWorldMap
from rrt_mavsim.message_types.msg_waypoints import MsgWaypoints_SFC

import numpy as np


class ViewManager:
    # arguments:
    # 1.
    def __init__(
        self,
        video_name: str,
        msg_plane: MsgPlane,
        world_map: MsgWorldMap=None,
        video: bool = False,
        data: bool = False,
        pathPlot: bool = False,
        sensors: bool = False,
        animation: bool = False,
        save_plots: bool = False,
        draw_trajectory: bool = False,
    ):
        self.video_flag = video
        self.data_plot_flag = data
        self.path_plot_flag = pathPlot
        self.sensor_plot_flag = sensors
        self.animation_flag = animation
        self.save_plots_flag = save_plots
        self.world_map = world_map
        self.msg_plane = msg_plane
        # initialize video
        # initialize the other visualization
        if self.animation_flag or self.data_plot_flag or self.sensor_plot_flag:
            self.app = pg.QtWidgets.QApplication([])
            if self.animation_flag:
                self.quadplane_view = QuadplaneViewer(
                    app=self.app,
                    dt=SIM.ts_simulation,
                    plot_period=SIM.ts_plot_refresh,
                    grid_on=False,
                    worldMap=world_map,
                    msg_plane=msg_plane,
                )
            if self.data_plot_flag:
                self.data_view = DataViewer(
                    app=self.app,
                    dt=SIM.ts_simulation,
                    plot_period=SIM.ts_plot_refresh,
                    data_recording_period=SIM.ts_plot_record_data,
                    time_window_length=30,
                )
            if self.path_plot_flag:
                self.autopilotDataView = AutopilotDataViewer(
                    app=self.app,
                    Ts=SIM.ts_simulation,
                    time_window_length=30,
                    plot_period=SIM.ts_plot_refresh,
                    data_recording_period=SIM.ts_plot_record_data,
                    plots_per_row=4,
                )

            if self.sensor_plot_flag:
                self.sensor_view = SensorViewer(
                    app=self.app,
                    dt=SIM.ts_simulation,
                    plot_period=SIM.ts_plot_refresh,
                    data_recording_period=SIM.ts_plot_record_data,
                    time_window_length=30,
                )

    def update(
        self,
        sim_time: float,
        true_state: MsgState,
        estimated_state: MsgState,
        commanded_state: MsgState,
        delta: MsgDelta,
        measurements: MsgSensors,
        integrator: MsgIntegrator,
        trajectory: MsgTrajectory,
    ):
        if self.animation_flag:
            self.quadplane_view.update(true_state)
        if self.data_plot_flag:
            self.data_view.update(
                true_state,  # true states
                estimated_state,  # estimated states
                commanded_state,  # commanded states
                delta,
            )  # inputs to aircraft
        if self.path_plot_flag:
            self.autopilotDataView.update(
                true_state=true_state, trajectory=trajectory, integrator=integrator
            )
        if self.sensor_plot_flag:
            self.sensor_view.update(measurements)
        if self.animation_flag or self.data_plot_flag or self.sensor_plot_flag:
            self.app.processEvents()

    def drawWaypoints(
        self,
        waypoints: MsgWaypoints_SFC,
        color="r",
    ):
        self.quadplane_view.drawWaypoints(
            waypoints=waypoints, plane=self.msg_plane, lineColor=color
        )

    def close(self, dataplot_name: str, sensorplot_name: str):
        # Save an Image of the Plot
        pass

    # creates the function to  draw a trajectory
    def drawTrajectory(
        self,
        controlPoints: np.ndarray,
        sampledPoints_spline: np.ndarray,
        lineColor: np.ndarray,
        lineWidth: float,
        pointWidth: float,
    ):
        self.quadplane_view.drawTrajectory(
            controlPoints=controlPoints,
            sampledPoints_spline=sampledPoints_spline,
            lineColor=lineColor,
            lineWidth=lineWidth,
            pointWidth=pointWidth,
        )

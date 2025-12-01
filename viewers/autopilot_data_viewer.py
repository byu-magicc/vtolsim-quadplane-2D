import pyqtgraph as pg
from viewers.plotter import Plotter
from message_types.msg_state import MsgState
from message_types.msg_trajectory import MsgTrajectory
from message_types.msg_integrator import MsgIntegrator
from tools.gamma import *


class AutopilotDataViewer:

    def __init__(self,
                 app:  pg.QtWidgets.QApplication,
                 Ts: float,
                 time_window_length: int,
                 plot_period: float,
                 data_recording_period: float,
                 plots_per_row: int):
        
        self.app = app
        self.Ts = Ts
        self.time_window_length = time_window_length
        self.plot_period = plot_period
        self.data_recording_period = data_recording_period
        self.data_recording_delay = 0
        self.plot_delay = 0
        self.plots_per_row = plots_per_row
        self.data_window_length= time_window_length/data_recording_period


        self.time = 0.0


        self.plotter = Plotter(app=app,
                               plots_per_row=self.plots_per_row)
        
        truth_color = (0,255,0)

        #creates the lists for the plots
        self.plot_id = [['north error', 'altitude error', 'north vel error', 'altitude vel error'], #Row 1
                   ['north accel ref', 'altitude accel ref', 'gamma ref', 'gamma error'], #Row 2
                   ['north integrator', 'altitude integrator', 'theta', 'theta error']] #Row 3
        
        self.plot_units = [['m', 'm', 'm/s', 'm/s'],
                      ['m/s^2', 'm/s^2', 'deg', 'deg'],
                      ['m', 'm', 'deg', 'deg']]

        #-------------------------------------First Row-------------------------------------------------
        
        #iterates over all of the rows
        for row, unit_row in zip(self.plot_id, self.plot_units):

            #iterates over each id and unit
            for id, unit in zip(row, unit_row):

                self.plotter.create_plot_widget(plot_id=id,
                                                xlabel='Time (s)',
                                                ylabel=id + ' (' + unit + ')' )
                
                self.plotter.create_data_set(plot_id=id,
                                             data_label=id,
                                             data_color=truth_color)
        

        self.plotter.show_window()

    def update(self,
               true_state: MsgState,
               trajectory: MsgTrajectory,
               integrator: MsgIntegrator):
        
        if self.data_recording_delay >= self.data_recording_period:
            self.update_data(true_state=true_state, trajectory=trajectory, integrator=integrator)
            self.data_recording_delay = 0
        if self.plot_delay >= self.plot_period:
            self.update_plot()
            self.plot_delay = 0

        self.plot_delay += self.Ts
        self.data_recording_delay += self.Ts
        self.time += self.Ts

    def update_data(self,
                    true_state: MsgState,
                    trajectory: MsgTrajectory,
                    integrator: MsgIntegrator):
        

        #creates the list of all of the values
        pos_error = trajectory.pos - true_state.pos_2D
        north_error = pos_error.item(0)
        altitude_error = pos_error.item(1)

        vel_error = trajectory.vel - true_state.vel_2D
        north_dot_error = vel_error.item(0)
        altitude_dot_error = vel_error.item(1)

        accel_ref = trajectory.accel
        north_ddot_ref = accel_ref.item(0)
        altitude_ddot_ref = accel_ref.item(1)

        #gets the gamma ref
        gamma_ref = getGamma(state_ref=trajectory)

        #gets the gamma from the true state
        integration = integrator.getIntegrator()
        integration_north = integration.item(0)
        integration_east = integration.item(1)

        theta = true_state.theta

        theta_error = trajectory.pitch - theta


        yValues = [[north_error, altitude_error, north_dot_error, altitude_dot_error],
                   [north_ddot_ref, altitude_ddot_ref, gamma_ref, 0.0],
                   [integration_north, integration_east, theta, theta_error]]



        #'''
        #iterates over all of the rows
        for row, yValueRow in zip(self.plot_id, yValues):

            #iterates over each id and unit
            for id, yValue in zip(row, yValueRow):

                self.plotter.add_data_point(plot_id=id,
                                            data_label=id,
                                            xvalue=self.time,
                                            yvalue=yValue)
                
                potato = 0
        #'''


    def update_plot(self):
        self.plotter.update_plots()
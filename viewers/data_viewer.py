import numpy as np
np.errstate(overflow="ignore")
from viewers.plotter import Plotter
from tools.wrap import wrap
from tools.rotations import rotation_to_euler
from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta


#creates the data viewer class
class DataViewer:
    def __init__(self, app,  dt = 0.01,
                 time_window_length = 30, # number of data points plotted at a time
                 plot_period = 0.2, # time interval between a plot update
                 data_recording_period = 0.1): # time interval between recording a data update
        self._dt = dt
        self._data_window_length= time_window_length/data_recording_period
        self._update_counter = 0
        self._plots_per_row = 3
        self._plotter = Plotter(app=app, plots_per_row=self._plots_per_row)  # plot last time_window seconds of data
        self._plot_period = plot_period
        self._data_recording_period = data_recording_period
        self._plot_delay = 0
        self._data_recording_delay = 0
        self._time = 0
        #define colors
        truth_color = (0,255,0)
        estimate_color = (255,0,0)
        command_color = (100,100,255)
        # define row layout        
        #-------------------------------------First Row-------------------------------------------------
        self._plotter.create_plot_widget(plot_id='north', xlabel='Time (s)', ylabel='north (m)',
                                        window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='altitude', xlabel='Time (s)', ylabel='altitude (m)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='pitch', xlabel='Time (s)', ylabel='pitch (deg)',
                                       window_length=self._data_window_length)
        #north data sets
        self._plotter.create_data_set(plot_id="north", data_label="north", data_color=truth_color)
        self._plotter.create_data_set(plot_id="north", data_label="north_e", data_color=estimate_color) 
        self._plotter.create_data_set(plot_id="north", data_label="north_c", data_color=command_color) 
        #altitude portion
        self._plotter.create_data_set(plot_id="altitude", data_label="altitude", data_color=truth_color)
        self._plotter.create_data_set(plot_id="altitude", data_label="altitude_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="altitude", data_label="altitude_c", data_color=command_color)
        #theta portion
        self._plotter.create_data_set(plot_id="pitch", data_label="pitch", data_color=truth_color)
        self._plotter.create_data_set(plot_id="pitch", data_label="pitch_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="pitch", data_label="pitch_c", data_color=command_color)
        #----------------------------------Second Row--------------------------------------------------
        self._plotter.create_plot_widget(plot_id='u', xlabel='Time (s)', ylabel='u (m/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='w', xlabel='Time (s)', ylabel='w (m/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='q', xlabel='Time (s)', ylabel='q (deg/s)',
                                       window_length=self._data_window_length)
        #u data sets
        self._plotter.create_data_set(plot_id="u", data_label="u", data_color=truth_color)
        self._plotter.create_data_set(plot_id="u", data_label="u_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="u", data_label="u_c", data_color=command_color)
        #w portion
        self._plotter.create_data_set(plot_id="w", data_label="w", data_color=truth_color)
        self._plotter.create_data_set(plot_id="w", data_label="w_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="w", data_label="w_c", data_color=command_color)
        #q portion
        self._plotter.create_data_set(plot_id="q", data_label="q", data_color=truth_color)
        self._plotter.create_data_set(plot_id="q", data_label="q_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="q", data_label="q_c", data_color=command_color)
        #----------------------------------------third row----------------------------------------------
        self._plotter.create_plot_widget(plot_id='Va', xlabel='Time (s)', ylabel='Va (m/s)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='alpha', xlabel='Time (s)', ylabel='alpha (deg)',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='elevator', xlabel='Time (s)', ylabel='elevator',
                                       window_length=self._data_window_length)
        #Va airspeed data sets
        self._plotter.create_data_set(plot_id="Va", data_label="Va", data_color=truth_color)
        self._plotter.create_data_set(plot_id="Va", data_label="Va_e", data_color=estimate_color)
        self._plotter.create_data_set(plot_id="Va", data_label="Va_c", data_color=command_color)
        #alpha portion
        self._plotter.create_data_set(plot_id="alpha", data_label="alpha", data_color=truth_color)
        self._plotter.create_data_set(plot_id="alpha", data_label="alpha_e", data_color=estimate_color)        
        #elevator portion
        self._plotter.create_data_set(plot_id="elevator", data_label="elevator", data_color=truth_color)
        #-------------------------------------fourth row---------------------------------------------------
        self._plotter.create_plot_widget(plot_id='delta_t', xlabel='Time (s)', ylabel='delta_t',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='delta_f', xlabel='Time (s)', ylabel='delta_f',
                                       window_length=self._data_window_length)
        self._plotter.create_plot_widget(plot_id='delta_r', xlabel='Time (s)', ylabel='delta_r',
                                       window_length=self._data_window_length)
        #delta_t potion
        self._plotter.create_data_set(plot_id="delta_t", data_label="delta_t", data_color=truth_color)
        #delta_f portion
        self._plotter.create_data_set(plot_id="delta_f", data_label="delta_f", data_color=truth_color)
        #delta_+r portion
        self._plotter.create_data_set(plot_id="delta_r", data_label="delta_r", data_color=truth_color)
        self._plotter.show_window()

    def update(self, true_state: MsgState, estimated_state: MsgState, commanded_state: MsgState, delta: MsgDelta):
        if self._data_recording_delay >= self._data_recording_period:
            self.__update_data(true_state, estimated_state, commanded_state, delta, self._time)
            self._data_recording_delay = 0
        if self._plot_delay >= self._plot_period:
            self.__update_plot()
            self._plot_delay = 0
        self._plot_delay += self._dt
        self._data_recording_delay += self._dt
        self._time += self._dt
        
    def __update_data(self, true_state: MsgState, estimated_state: MsgState, commanded_state: MsgState, delta: MsgDelta, currentTime):
        #add the true state data
        if true_state != None:
            phi, theta, psi = rotation_to_euler(true_state.R)
            #adds the actual positions
            self._plotter.add_data_point(plot_id='north', data_label='north', xvalue=currentTime, yvalue=true_state.pos[0,0])
            self._plotter.add_data_point(plot_id='altitude', data_label='altitude', xvalue=currentTime, yvalue=-true_state.pos[2,0])
            #adds the body frame velocities
            self._plotter.add_data_point(plot_id='u', data_label='u', xvalue=currentTime, yvalue=true_state.vel[0,0])
            self._plotter.add_data_point(plot_id='w', data_label='w', xvalue=currentTime, yvalue=true_state.vel[2,0])
            #adds the pitchw
            self._plotter.add_data_point(plot_id='pitch', data_label='pitch', xvalue=currentTime, yvalue=self.__rad_to_deg(theta))
            #adds the q
            self._plotter.add_data_point(plot_id='q', data_label='q', xvalue=currentTime, yvalue=self.__rad_to_deg(true_state.omega[1,0]))
            #adds the Airspeed, alpha
            self._plotter.add_data_point(plot_id='Va', data_label='Va', xvalue=currentTime, yvalue=true_state.Va)
            self._plotter.add_data_point(plot_id='alpha', data_label='alpha', xvalue=currentTime, yvalue=true_state.alpha)
            #adds elevator
            self._plotter.add_data_point(plot_id='elevator', data_label='elevator', xvalue=currentTime, yvalue=delta.elevator)
            #adds motor inputs
            self._plotter.add_data_point(plot_id='delta_t', data_label='delta_t', xvalue=currentTime, yvalue=delta.throttle_thrust)
            self._plotter.add_data_point(plot_id='delta_f', data_label='delta_f', xvalue=currentTime, yvalue=delta.throttle_front)
            self._plotter.add_data_point(plot_id='delta_r', data_label='delta_r', xvalue=currentTime, yvalue=delta.throttle_rear)

        #add the estimated state data
        if estimated_state != None:
            phi, theta, psi = rotation_to_euler(estimated_state.R)
            #adds the actual positions
            self._plotter.add_data_point(plot_id='north', data_label='north', xvalue=currentTime, yvalue=estimated_state.pos[0,0])
            self._plotter.add_data_point(plot_id='altitude', data_label='altitude', xvalue=currentTime, yvalue=-estimated_state.pos[2,0])
            #adds the body frame velocities
            self._plotter.add_data_point(plot_id='u', data_label='u', xvalue=currentTime, yvalue=estimated_state.vel[0,0])
            self._plotter.add_data_point(plot_id='w', data_label='w', xvalue=currentTime, yvalue=estimated_state.vel[2,0])
            #adds the pitchw
            self._plotter.add_data_point(plot_id='pitch', data_label='pitch', xvalue=currentTime, yvalue=self.__rad_to_deg(theta))
            #adds the q
            self._plotter.add_data_point(plot_id='q', data_label='q', xvalue=currentTime, yvalue=self.__rad_to_deg(estimated_state.omega[1,0]))
            #adds the Airspeed, alpha
            self._plotter.add_data_point(plot_id='Va', data_label='Va', xvalue=currentTime, yvalue=estimated_state.Va)
            self._plotter.add_data_point(plot_id='alpha', data_label='alpha', xvalue=currentTime, yvalue=estimated_state.alpha)

        #add the commanded state data
        if commanded_state != None:
            phi, theta, psi = rotation_to_euler(commanded_state.R)
            #adds the actual positions
            self._plotter.add_data_point(plot_id='north', data_label='north', xvalue=currentTime, yvalue=commanded_state.pos[0,0])
            self._plotter.add_data_point(plot_id='altitude', data_label='altitude', xvalue=currentTime, yvalue=-commanded_state.pos[2,0])
            #adds the body frame velocities
            self._plotter.add_data_point(plot_id='u', data_label='u', xvalue=currentTime, yvalue=commanded_state.vel[0,0])
            self._plotter.add_data_point(plot_id='w', data_label='w', xvalue=currentTime, yvalue=commanded_state.vel[2,0])
            #adds the pitchw
            self._plotter.add_data_point(plot_id='pitch', data_label='pitch', xvalue=currentTime, yvalue=self.__rad_to_deg(theta))
            #adds the q
            self._plotter.add_data_point(plot_id='q', data_label='q', xvalue=currentTime, yvalue=self.__rad_to_deg(commanded_state.omega[1,0]))
            #adds the Airspeed, alpha
            self._plotter.add_data_point(plot_id='Va', data_label='Va', xvalue=currentTime, yvalue=commanded_state.Va)

    def process_app(self):
        self._plotter.process_app(0)

    def __update_plot(self):
        self._plotter.update_plots()

    def close_data_viewer(self):
        self._plotter.close_window()

    def save_plot_image(self, plot_name):
        self._plotter.save_image(plot_name)

    def __rad_to_deg(self, radians):
        rad = wrap(radians,0)
        return rad*180/np.pi



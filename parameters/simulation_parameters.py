import numpy as np

######################################################################################
                #   sample times, etc
######################################################################################
ts_simulation = 0.01  # smallest time step for simulation
start_time = 0.0  # start time for simulation
end_time = 30.

ts_plot_refresh = 0.5  # seconds between each plot update
ts_plot_record_data = 0.1 # seconds between each time data is recorded for plots
#ts_plotting = 1.0  # refresh rate for plots
ts_video = 0.1  # write rate for video
ts_control = ts_simulation  # sample rate for the controller

#saves the sleeping time
sleep_time = 0.01

show_airsim = False



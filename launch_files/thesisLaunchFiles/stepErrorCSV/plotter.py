import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

df0=pd.read_csv('times.csv', header=None)
df1=pd.read_csv('positionsActual.csv', header=None)
df2=pd.read_csv('velocitiesActual.csv', header=None)
df3=pd.read_csv('positionsRef.csv', header=None)
df4=pd.read_csv('velocitiesRef.csv', header=None)
df5=pd.read_csv('deltas.csv', header=None)
df6=pd.read_csv('angles.csv',header=None)

#gets the time array
time_array = df0[0].to_numpy()

#gets the actual positions
pos_north_actual = df1[0].to_numpy()
pos_down_actual = df1[1].to_numpy()

#gets the actual velocities
vel_north_actual = df2[0].to_numpy()
vel_down_actual = df2[1].to_numpy()

#gets the reference positions and velocities
pos_north_ref = df3[0].to_numpy()
pos_down_ref = df3[1].to_numpy()

vel_north_ref = df4[0].to_numpy()
vel_down_ref = df4[1].to_numpy()

#gets the positional errors
pos_north_error = pos_north_ref - pos_north_actual
pos_down_error = pos_down_ref - pos_down_actual
pos_error_mag = [np.sqrt(tempNorthError**2 + tempDownError**2) for tempNorthError, tempDownError in zip(pos_north_error, pos_down_error)]

#gets the velocity errors
vel_north_error = vel_north_ref - vel_north_actual
vel_down_error = vel_down_ref - vel_down_actual
vel_error_mag = [np.sqrt(tempNorthError**2 + tempDownError**2) for tempNorthError, tempDownError in zip(vel_north_error, vel_down_error)]


#getst the deltas
delta_e = df5[0].to_numpy()
delta_t_front = df5[1].to_numpy()
delta_t_rear = df5[2].to_numpy()
delta_t_thrust = df5[3].to_numpy()

#gets the angles
theta = df6[0].to_numpy()
gamma = df6[1].to_numpy()
gamma_ref = df6[2].to_numpy()
alpha = df6[3].to_numpy()

fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

#plots on the positional errors
ax1.plot(time_array, pos_north_error, label='North Position Error')
ax1.plot(time_array, pos_down_error, label='Down Position Error')
ax1.plot(time_array, pos_error_mag, label='Position Error Magnitude')
ax1.legend()
ax1.grid(True)

ax2.plot(time_array, vel_north_error, label='North Velocity Error')
ax2.plot(time_array, vel_down_error, label='Down Velocity Error')
ax2.plot(time_array, vel_error_mag, label='Velocity Error Magnitude')
ax2.legend()
ax2.grid(True)

plt.show()

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)

ax1.plot(time_array, delta_t_front, label='Delta t Front')
ax1.plot(time_array, delta_t_rear, label='Delta t Rear')
ax1.legend()
ax1.grid(True)

ax2.plot(time_array, delta_t_thrust, label='Delta t Thrust')
ax2.legend()
ax2.grid(True)

ax3.plot(time_array, delta_e, label='Delta elevator')
ax3.legend()
ax3.grid(True)

plt.show()

fig, (ax1, ax2, ax3) = plt.subplots(3,1,sharex=True)

ax1.plot(time_array, np.degrees(theta), label='Pitch (deg)')
ax1.grid(True)
ax1.legend()

ax2.plot(time_array, np.degrees(gamma), label='flight path angle (deg)')
ax2.plot(time_array, np.degrees(gamma_ref), label='flight path angle ref (deg)')
ax2.grid(True)
ax2.legend()

ax3.plot(time_array, np.degrees(alpha), label='angle of attack (deg)')
ax3.grid(True)
ax3.legend()

plt.show()


testPoint = 0

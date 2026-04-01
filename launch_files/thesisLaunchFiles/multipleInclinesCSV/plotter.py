import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from pathlib import Path
import matplotlib.font_manager as fm


plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
    "mathtext.fontset": "stix",   # math matches Times style
    "axes.titlesize": 12,
    "axes.labelsize": 10,
    "font.size": 10,
    "legend.fontsize": 20,
    "xtick.labelsize": 9,
    "ytick.labelsize": 9,
})

df_time = pd.read_csv('times.csv', header=None)
time_array = df_time[0].to_numpy()

#deltas section
df_deltas = pd.read_csv('deltas.csv', header=None)
delta_e = df_deltas[0].to_numpy()
delta_t_front = df_deltas[1].to_numpy()
delta_t_rear = df_deltas[2].to_numpy()
delta_t_thrust = df_deltas[3].to_numpy()

fig, ax = plt.subplots(figsize=(8,4))
ax.plot(time_array, delta_e, label='elevator')
ax.plot(time_array, delta_t_front, label='front')
ax.plot(time_array, delta_t_rear, label='rear')
ax.plot(time_array, delta_t_thrust, label='thrust')
font = fm.FontProperties(family='serif', size=14)
ax.legend(prop=font)
ax.set_title("Multiple Ascensions Control Inputs", fontsize=14)
plt.show()

df_theta_ref = pd.read_csv('thetaRefArray.csv', header=None)
thetaRef = df_theta_ref[0].to_numpy()

plt.figure(1)
plt.plot(time_array, thetaRef, label='Theta ref')
plt.legend()
plt.show()

df_theta_constraints = pd.read_csv('constraintsArray.csv', header=None)
lowerConstraints = df_theta_constraints[0].to_numpy()
upperConstraints = df_theta_constraints[1].to_numpy()

plt.figure(2)
plt.plot(time_array, lowerConstraints, label='lowerConstraints')
plt.plot(time_array, upperConstraints, label='upperConstraints')
plt.legend()
plt.show()

df3 = pd.read_csv('ActualPositions.csv', header=None)
actual_northPosition = df3[0].to_numpy()
actual_downPosition = df3[1].to_numpy()

df4 = pd.read_csv('desiredPositions.csv', header=None)
desired_northPosition = df4[0].to_numpy()
desired_downPosition = df4[1].to_numpy()

df5 = pd.read_csv('actualVelocities.csv', header=None)
actual_northVel = df5[0].to_numpy()
actual_downVel = df5[1].to_numpy()

df6 = pd.read_csv('desiredVelocities.csv', header=None)
desired_northVel = df6[0].to_numpy()
desired_downVel = df6[1].to_numpy()

df7 = pd.read_csv('ControlPoints.csv', header=None)
controlPoints_northPosition = df7[0].to_numpy()
controlPoints_downPosition = df7[1].to_numpy()

#gets the positional errors
pos_north_error = desired_northPosition - actual_northPosition
pos_down_error = desired_downPosition - actual_downPosition

#gets the velocity errors
vel_north_error = desired_northVel - actual_northVel
vel_down_error = desired_downVel - actual_downVel

fig, (ax1, ax2) = plt.subplots(2,1, sharex=True)
ax1.plot(time_array, pos_north_error, label='N Pos Error')
ax1.plot(time_array, pos_down_error, label='D Pos Error')
font = fm.FontProperties(family='serif', size=14)
ax1.legend(prop=font)
ax1.grid(True)
ax1.set_title("Position Error", fontsize=14)
ax1.set_ylabel('Position Error (m)', fontsize=12)


ax2.plot(time_array, vel_north_error, label='N Vel Error')
ax2.plot(time_array, vel_down_error, label='D Vel Error')
font = fm.FontProperties(family='serif', size=14)
ax2.legend(prop=font)
ax2.grid(True)
ax2.set_title("Velocity Error", fontsize=14)
ax2.set_xlabel('time (s)', fontsize=12)
ax2.set_ylabel('Velocity Error (m/s)', fontsize=12)

plt.show()


fig, (ax1, ax2) = plt.subplots(2,1, sharex=True)
ax1.plot(time_array, desired_northPosition, label='North Position Desired')
ax1.plot(time_array, actual_northPosition, label='North Position Actual')
ax1.legend()
ax1.grid(True)
ax1.set_ylabel('Position (m)')

ax2.plot(time_array, desired_downPosition, label='Down Position Desired')
ax2.plot(time_array, actual_downPosition, label='Down Position Actual')
ax2.legend()
ax2.grid(True)
ax2.set_ylabel('Position (m)')

ax2.set_xlabel('time (s)')

plt.show()

fig, (ax1, ax2) = plt.subplots(2,1, sharex=True)
ax1.plot(time_array, desired_northVel, label='North Vel Desired')
ax1.plot(time_array, actual_northVel, label='North Vel Actual')
ax1.legend()
ax1.grid(True)
ax1.set_ylabel('Vel (m/s)')

ax2.plot(time_array, desired_downVel, label='Down Vel Desired')
ax2.plot(time_array, actual_downVel, label='Down Vel Actual')
ax2.legend()
ax2.grid(True)
ax2.set_ylabel('Vel (m/s)')

ax2.set_xlabel('time (s)')

plt.show()



fig, (ax1) = plt.subplots(1,1, sharex=True)
ax1.plot(desired_northPosition, -1*desired_downPosition, label='Position Trajectory')
#ax1.scatter(controlPoints_northPosition, -1*controlPoints_downPosition, label='Control Points', linewidths=0.5)
ax1.legend()
ax1.set_title('Position Trajectory')
ax1.set_aspect('equal', adjustable='box')
ax1.set_ylabel('Altitude Position (m)')

plt.show()

fig, (ax1, ax2, ax3) = plt.subplots(3,1, sharex=True)
ax1.plot(desired_northPosition, -1*desired_downPosition, label='Position Trajectory')
ax1.legend()
ax1.set_title('Position Trajectory')
ax1.set_aspect('equal', adjustable='box')
ax1.set_ylabel('Altitude Position (m)')

ax2.plot(desired_northPosition, desired_northVel, label='North Velocity Trajectory')
ax2.legend()
ax2.set_title('North Velocity Trajectory')
ax2.set_ylabel('North Velocity (m/s)')

ax3.plot(desired_northPosition, desired_downVel, label='Down Velocity Trajectory')
ax3.legend()
ax3.set_title('Down Velocity Trajectory')
ax3.set_ylabel('Down Velocity (m/s)')
ax3.set_xlabel('North Position (meters)')

plt.show()

#generates the actual versus desired positions
fig, ax = plt.subplots(1,1,sharex=True)
ax.plot(desired_northPosition, -1*desired_downPosition, label='Desired Trajectory')
ax.plot(actual_northPosition, -1*actual_downPosition, label='Actual Trajectory')
ax.set_xlabel('North Position (m)')
ax.set_ylabel('Altitude (m)')
ax.set_title('Actual versus desired trajectory')
ax.set_aspect('equal', adjustable='box')
ax.legend()

plt.show()


df8 = pd.read_csv('forcesActual.csv', header=None)
northForce_actual = df8[0].to_numpy()
downForce_actual = df8[1].to_numpy()

df9 = pd.read_csv('forcesDesired.csv', header=None)
northForce_desired = df9[0].to_numpy()
downForce_desired = df9[1].to_numpy()

northForce_error = northForce_desired - northForce_actual
downForce_error = downForce_desired - downForce_actual

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)
ax1.plot(time_array, northForce_error, label='North Force Error')
ax1.plot(time_array, downForce_error, label='Down Force Error')
ax1.legend()
ax1.grid(True)


ax2.plot(time_array, northForce_desired, label='North Force Desired')
ax2.plot(time_array, northForce_actual, label='North Force Actual')
ax2.legend()
ax2.grid(True)


ax3.plot(time_array, downForce_desired, label='Down Force Desired')
ax3.plot(time_array, downForce_actual, label='Down Force Actual')
ax3.set_xlabel('Time (s)')
ax3.legend()
ax3.grid(True)

plt.show()

df10=pd.read_csv('angles.csv',header=None)

#gets the angles
theta = df10[0].to_numpy()
gamma = df10[1].to_numpy()
gamma_ref = df10[2].to_numpy()
alpha = df10[3].to_numpy()


fig, (ax1, ax2, ax3) = plt.subplots(3,1,sharex=True)

ax1.plot(time_array, np.degrees(theta), label='Pitch (deg)')
ax1.grid(True)
ax1.legend()
ax1.set_ylabel('Angle (degrees)')
ax1.set_title('Pitch')

ax2.plot(time_array, np.degrees(gamma), label='flight path angle (deg)')
ax2.plot(time_array, np.degrees(gamma_ref), label='flight path angle ref (deg)')
ax2.grid(True)
ax2.legend()
ax2.set_ylabel('Angle (degrees)')
ax2.set_title('Flight Path Angle')

ax3.plot(time_array, np.degrees(alpha), label='angle of attack (deg)')
ax3.grid(True)
ax3.legend()
ax3.set_xlabel('Time (seconds)')
ax3.set_ylabel('Angle (degrees)')
ax3.set_title('Angle of Attack')

plt.show()

testPoint = 0

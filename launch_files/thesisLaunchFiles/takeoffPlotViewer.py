import matplotlib.pyplot as plt
import pandas as pd
import numpy as np


df = pd.read_csv('launch_files/thesisLaunchFiles/takeoffCSV/times.csv', header=None)
df1 = pd.read_csv('launch_files/thesisLaunchFiles/takeoffCSV/ActualPositions.csv', header=None)
df2 = pd.read_csv('launch_files/thesisLaunchFiles/takeoffCSV/desiredPositions.csv', header=None)
df3 = pd.read_csv('launch_files/thesisLaunchFiles/takeoffCSV/desiredVelocities.csv', header=None)
df4 = pd.read_csv('launch_files/thesisLaunchFiles/takeoffCSV/desiredAccelerations.csv', header=None)
df5 = pd.read_csv('launch_files/thesisLaunchFiles/takeoffCSV/actualVelocities.csv', header=None)

#position section
t = df[0].to_numpy()
x_actual = df1[0].to_numpy()
z_actual = df1[1].to_numpy()

x_desired = df2[0].to_numpy()
z_desired = df2[1].to_numpy()

x_error = x_desired - x_actual
z_error = z_desired - z_actual

position_error = np.sqrt(x_error**2 + z_error**2)

#velocity section
x_dot_actual = df5[0].to_numpy()
z_dot_actual = df5[1].to_numpy()

x_dot_desired = df3[0].to_numpy()
z_dot_desired = df3[1].to_numpy()

x_dot_error = x_dot_desired - x_dot_actual
z_dot_error = z_dot_desired - z_dot_actual

velocity_error = np.sqrt(x_dot_error**2 + z_dot_error**2)



plt.figure(0)
plt.plot(t, x_actual, label='x actual')
plt.plot(t, x_desired, label='x desired')
plt.xlabel('time (s)')
plt.ylabel('position (m)')
plt.legend()
plt.show()


plt.figure(1)
plt.plot(t, z_actual, label='z actual')
plt.plot(t, z_desired, label='z desired')
plt.xlabel('time (s)')
plt.ylabel('position (m)')
plt.legend()
plt.show()

plt.figure(2)
plt.plot(t, position_error, label='Position Error Magnitude')
plt.xlabel('time (s)')
plt.ylabel('position (m)')
plt.legend()
plt.show()


plt.figure(3)
plt.plot(t, x_dot_actual, label='x dot actual')
plt.plot(t, x_dot_desired, label='x dot desired')
plt.xlabel('time (s)')
plt.ylabel('velocity (m)')
plt.legend()
plt.show()


plt.figure(4)
plt.plot(t, z_dot_actual, label='z dot actual')
plt.plot(t, z_dot_desired, label='z dot desired')
plt.xlabel('time (s)')
plt.ylabel('velocity (m)')
plt.legend()
plt.show()

plt.figure(5)
plt.plot(t, velocity_error, label='Velocity Error Magnitude')
plt.xlabel('time (s)')
plt.ylabel('velocity (m)')
plt.legend()
plt.show()



testPoint = 0

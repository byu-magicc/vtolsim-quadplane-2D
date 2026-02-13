
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np


df = pd.read_csv('launch_files/thesisLaunchFiles/landingCSV/times.csv', header=None)
df1 = pd.read_csv('launch_files/thesisLaunchFiles/landingCSV/ActualPositions.csv', header=None)
df2 = pd.read_csv('launch_files/thesisLaunchFiles/landingCSV/desiredPositions.csv', header=None)
df3 = pd.read_csv('launch_files/thesisLaunchFiles/landingCSV/desiredVelocities.csv', header=None)
df4 = pd.read_csv('launch_files/thesisLaunchFiles/landingCSV/desiredAccelerations.csv', header=None)
df5 = pd.read_csv('launch_files/thesisLaunchFiles/landingCSV/actualVelocities.csv', header=None)

#position section
t_landing = df[0].to_numpy()
x_actual_landing = df1[0].to_numpy()
z_actual_landing = df1[1].to_numpy()

x_desired_landing = df2[0].to_numpy()
z_desired_landing = df2[1].to_numpy()

x_error_landing = x_desired_landing - x_actual_landing
z_error_landing = z_desired_landing - z_actual_landing

position_error_landing = np.sqrt(x_error_landing**2 + z_error_landing**2)

#velocity section
x_dot_actual_landing = df5[0].to_numpy()
z_dot_actual_landing = df5[1].to_numpy()

x_dot_desired_landing = df3[0].to_numpy()
z_dot_desired_landing = df3[1].to_numpy()

x_dot_error_landing = x_dot_desired_landing - x_dot_actual_landing
z_dot_error_landing = z_dot_desired_landing - z_dot_actual_landing

velocity_error_landing = np.sqrt(x_dot_error_landing**2 + z_dot_error_landing**2)




plt.figure(1)
plt.plot(t_landing, z_actual_landing, label='z actual')
plt.plot(t_landing, z_desired_landing, label='z desired')
plt.xlabel('time (s)')
plt.ylabel('position (m)')
plt.legend()
plt.show()

plt.figure(2)
plt.plot(t_landing, position_error_landing, label='Position Error Magnitude')
plt.xlabel('time (s)')
plt.ylabel('position (m)')
plt.legend()
plt.show()


plt.figure(3)
plt.plot(t_landing, x_dot_actual_landing, label='x dot actual')
plt.plot(t_landing, x_dot_desired_landing, label='x dot desired')
plt.xlabel('time (s)')
plt.ylabel('velocity (m)')
plt.legend()
plt.show()


plt.figure(4)
plt.plot(t_landing, z_dot_actual_landing, label='z dot actual')
plt.plot(t_landing, z_dot_desired_landing, label='z dot desired')
plt.xlabel('time (s)')
plt.ylabel('velocity (m)')
plt.legend()
plt.show()

plt.figure(5)
plt.plot(t_landing, velocity_error_landing, label='Velocity Error Magnitude')
plt.xlabel('time (s)')
plt.ylabel('velocity (m)')
plt.legend()
plt.show()


df6 = pd.read_csv('launch_files/thesisLaunchFiles/takeoffCSV/times.csv', header=None)
df7 = pd.read_csv('launch_files/thesisLaunchFiles/takeoffCSV/ActualPositions.csv', header=None)
df8 = pd.read_csv('launch_files/thesisLaunchFiles/takeoffCSV/desiredPositions.csv', header=None)
df9 = pd.read_csv('launch_files/thesisLaunchFiles/takeoffCSV/desiredVelocities.csv', header=None)
df10 = pd.read_csv('launch_files/thesisLaunchFiles/takeoffCSV/desiredAccelerations.csv', header=None)
df11 = pd.read_csv('launch_files/thesisLaunchFiles/takeoffCSV/actualVelocities.csv', header=None)

#position section
t_takeoff = df6[0].to_numpy()
x_actual_takeoff = df7[0].to_numpy()
z_actual_takeoff = df7[1].to_numpy()

x_desired_takeoff = df8[0].to_numpy()
z_desired_takeoff = df8[1].to_numpy()

x_error_takeoff = x_desired_takeoff - x_actual_takeoff
z_error_takeoff = z_desired_takeoff - z_actual_takeoff

position_error_takeoff = np.sqrt(x_error_takeoff**2 + z_error_takeoff**2)

#velocity section
x_dot_actual_takeoff = df9[0].to_numpy()
z_dot_actual_takeoff = df9[1].to_numpy()

x_dot_desired_takeoff = df10[0].to_numpy()
z_dot_desired_takeoff = df10[1].to_numpy()

x_dot_error_takeoff = x_dot_desired_takeoff - x_dot_actual_takeoff
z_dot_error_takeoff = z_dot_desired_takeoff - z_dot_actual_takeoff

velocity_error_takeoff = np.sqrt(x_dot_error_takeoff**2 + z_dot_error_takeoff**2)

#plots the error comparison between takeoff and landingCSV
plt.figure(6)
plt.plot(t_takeoff, position_error_takeoff, label='position error takeoff')
plt.plot(t_landing, position_error_landing, label='position error landing')
plt.legend()
plt.show()


plt.figure(7)
plt.plot(t_takeoff, velocity_error_takeoff, label='velocity error takeoff')
plt.plot(t_landing, velocity_error_landing, label='velocity error landing')
plt.legend()
plt.show()

testPoint = 0

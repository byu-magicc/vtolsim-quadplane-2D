import matplotlib.pyplot as plt
import pandas as pd
import numpy as np


df = pd.read_csv('launch_files/thesisLaunchFiles/times.csv', header=None)
df1 = pd.read_csv('launch_files/thesisLaunchFiles/ActualPositions.csv', header=None)
df2 = pd.read_csv('launch_files/thesisLaunchFiles/desiredPositions.csv', header=None)

t = df[0].to_numpy()
x_actual = df1[0].to_numpy()
z_actual = df1[1].to_numpy()

x_desired = df2[0].to_numpy()
z_desired = df2[1].to_numpy()


x_error = x_desired - x_actual
z_error = z_desired - z_actual

position_error = np.sqrt(x_error**2 + z_error**2)

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




testPoint = 0
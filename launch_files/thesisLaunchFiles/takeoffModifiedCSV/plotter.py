import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from pathlib import Path

print(Path.cwd())


df_time = pd.read_csv('times.csv', header=None)
t = df_time[0].to_numpy()

#deltas section
df_deltas = pd.read_csv('deltas.csv', header=None)
delta_e = df_deltas[0].to_numpy()
delta_t_front = df_deltas[1].to_numpy()
delta_t_rear = df_deltas[2].to_numpy()
delta_t_thrust = df_deltas[3].to_numpy()

plt.figure(0)
plt.plot(t, delta_e, label='elevator')
plt.plot(t, delta_t_front, label='front')
plt.plot(t, delta_t_rear, label='rear')
plt.plot(t, delta_t_thrust, label='thrust')
plt.legend()
plt.show()



testPoint = 0

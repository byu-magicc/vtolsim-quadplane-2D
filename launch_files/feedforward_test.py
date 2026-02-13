#implements a test for the feedforward controller
import os, sys

from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))
import numpy as np

import matplotlib.pyplot as plt
from models.rotatingMassDynamics import rotatingMassDynamics
from controllers.feedforwardControl import feedForwardControl
import parameters.simulation_parameters as SIM
import parameters.anaconda_parameters as CONDA
from tools.signal_generator import SignalGenerator

rotatingMass = rotatingMassDynamics(alpha=1.0)
controller = feedForwardControl(kp=5.6,
                                kd=3.563,
                                Ts=SIM.ts_control,
                                Jy=CONDA.Jy,
                                u_max=10.0,
                                sigma=0.05)

signalGen = SignalGenerator(amplitude=1.0,
                            frequency=0.1)

theta_reference = []

theta_output = []


t = SIM.start_time
y = rotatingMass.h()

while t < SIM.end_time:

    reference = signalGen.step(time=t)
    theta_reference.append(reference)

    state = rotatingMass.state
    theta = state.item(0)
    theta_dot = state.item(1)
    theta_output.append(theta)

    tau=controller.update(state=theta,
                          state_dot=theta_dot,
                          state_ref=reference)
    
    u = np.array([[tau]])
    y = rotatingMass.update(u=u)

    t += SIM.ts_simulation


theta_reference = np.array(theta_reference)
theta_output = np.array(theta_output)


plt.figure(0)
plt.plot(theta_reference, color='blue', label='Theta Reference')
plt.plot(theta_output, color='red', label='Theta Output')
plt.legend()
plt.show()


potato = 0
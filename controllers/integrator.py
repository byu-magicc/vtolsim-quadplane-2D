"""
integrator
        1/20/2025 - RWB
"""
# import numpy as np 
# from tools.saturate import saturate


class Integrator:
    def __init__(self, ts_control):
        self.Ts = ts_control
        # initialize integrator and delay variable
        self.integrator = 0.
        self.signal_d1 = 0.

    def update(self, signal):
        self.integrator \
            += (self.Ts/2) * (signal + self.signal_d1)
        self.signal_d1 = signal
        return self.integrator


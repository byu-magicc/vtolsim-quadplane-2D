"""
    - Last Update:
        2/4/25 - RWB
"""
import matplotlib.pyplot as plt

class AlphaFilter:
    # an alpha-filter is a low-pass filter of the input:
    # y = LPF(u), with transfer function H(s) = a/(s+a)
    # if alpha = exp(-a*Ts), where Ts is a constant sample rate
    def __init__(self,
                 alpha=0.5,
                 y0=0.0,):
        self.alpha = alpha
        self.y = y0  # output of alpha-filter.  Initialize with y0

    def update(self, u):
        self.y = self.alpha * self.y + (1-self.alpha) * u
        return self.y


class BetaFilter:
    # a beta-filter is a dirty derivative of the input:
    # y ~ ydot = DD(u), with transfer function H(s) = s/(s+a)
    # if beta = exp(-a*Ts), where Ts is a constant sample rate
    def __init__(self,
                 beta:float=0.5,
                 Ts:float=0.01,
                 y0:float=0.0):
        self.beta = beta
        self.Ts = Ts  # sample rate
        self.y = y0  # output of beta-filter.  Initialize with y0
        self.u_d1 = 0.  # input delayed by one sample

    def update(self, u:float):
        udot = (u - self.u_d1) / self.Ts
        self.y = self.beta * self.y + (1-self.beta) * udot
        self.u_d1 = u
        return self.y

# if __name__ == "__main__":
#     # instantiate the system
#     input = signals(amplitude=2.0, frequency=2.0)
#     Ts = 0.01

#     # main simulation loop
#     sim_time = -1.0
#     time = [sim_time]
#     #output = [input.sinusoid(sim_time)]
#     #output = [input.step(sim_time)]
#     #output = [input.impulse(sim_time)]
#     #output = [input.doublet(sim_time)]
#     #output = [input.random(sim_time)]
#     #output = [input.square(sim_time)]
#     output = [input.sawtooth(sim_time)]
#     while sim_time < 10.0:
#         #y = input.sinusoid(sim_time)
#         #y = input.step(sim_time)
#         #y = input.impulse(sim_time)
#         #y = input.doublet(sim_time)
#         #y = input.random(sim_time)
#         #y = input.square(sim_time)
#         y = input.sawtooth(sim_time)

#         sim_time += Ts   # increment the simulation time

#         # update date for plotting
#         time.append(sim_time)
#         output.append(y)

#     # plot output vs time
#     plt.plot(time, output)
#     plt.show()



import numpy as np
import parameters.rotatingMassParam as P


class rotatingMassDynamics:

    #alpha: the random variable magnitude
    def __init__(self,
                 alpha: float = 0.0):

        #creates the state array
        self.state = np.array([[P.theta0],
                               [P.thetadot0]])
        

        self.Ts = P.Ts

        #gets the rotational inertial
        self.Jy = P.Jy * (1.+alpha*(2.*np.random.rand()-1.))

        self.torque_limit = P.tau_max

        #sets the u limits vector
        self.u_limits = np.array([[self.torque_limit]])

    #u: the control input (just moment M in this case as a vector)
    def update(self,
               u: np.ndarray):
        
        u_sat = saturate_vector(u=u, limits=self.u_limits)
        self.rk4_step(u=u_sat)
        y = self.h()
        return y
    
    #the f update function
    def f(self,
          state: np.ndarray,
          u: np.ndarray):
        
        #sets the state update matrices
        A = np.array([[0.0, 1.0],
                      [0.0, 0.0]])
        
        B = np.array([[0.0],
                      [1.0/self.Jy]])
        
        #gets x_dot using the update function
        x_dot = A @ state + B @ u
        
        return x_dot

    #go from state to output sensor
    def h(self):

        return self.state

    #defines the rk4 step
    def rk4_step(self,
                 u: np.ndarray):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        F1 = self.f(self.state, u)
        F2 = self.f(self.state + self.Ts / 2 * F1, u)
        F3 = self.f(self.state + self.Ts / 2 * F2, u)
        F4 = self.f(self.state + self.Ts * F3, u)
        self.state = self.state + self.Ts / 6 * (F1 + 2*F2 + 2*F3 + F4)





#defines the saturation function as vector form for both the limits and the u
def saturate_vector(u: np.ndarray,
             limits: np.ndarray):
    
    #iterates over all of the items in u
    numItems = np.size(u)

    outputList = []

    for i in range(numItems):
        #gets the  current item
        u_item_temp = u.item(i)
        limit_item_temp = limits.item(i)
        #gets the saturated version of that value
        u_sat = saturate_scalar(u=u_item_temp, limit=limit_item_temp)
        outputList.append(u_sat)

    #creates the vertical vector from the output list
    u_output = np.array(outputList).reshape(-1,1)
    return u_output

#the individual scalar version of the saturation
def saturate_scalar(u: float,
                    limit: float):
    #case the magnitude of u is too great
    if np.abs(u) > np.abs(limit):
        u = limit * np.sign(u)
    return u
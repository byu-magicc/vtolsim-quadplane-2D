"""
    - Last Update:
        1/24/2025  - RWB
"""
import numpy as np 
from scipy.optimize import minimize
from models.quadplane_dynamics import QuadplaneDynamics
from message_types.msg_delta import MsgDelta
from message_types.msg_state import MsgState

# from scipy.linalg import solve_continuous_are, inv
# from tools.rotations import rotation_to_euler, euler_to_rotation
# import parameters.anaconda_parameters as PARAM
# #from controllers.integrator import Integrator
# from message_types.msg_trajectory import MsgTrajectory

class ControlAllocator:
    def __init__(self, ts_control:float):
        self.Ts = ts_control
        self.delta = MsgDelta()
        self.quadplane_model = QuadplaneDynamics(ts_control)

    def update(self, 
               wrench_des: np.ndarray, 
               state: MsgState,
               ):
        
        elevator0 = 0.
        # define inequality constraints
        cons = ([
            {'type': 'ineq', #>0
                'fun': lambda x: np.array([# elevator constraint
                    np.radians(45)-x[0], # theta <= thetabar
                    x[0]+np.radians(45), # theta >= -thetabar
                    ]),
                'jac': lambda x: np.array([
                    [-1.],
                    [1.],
                    ])
            }
        ])
        result = minimize(wrench_objective_fun, 
                          elevator0, 
                          method='SLSQP', 
                          args = (self.quadplane_model, wrench_des, state),
                          constraints=cons, 
                          options={'ftol': 1e-10, 'disp': True, 'maxiter': 1000})
        self.delta.elevator = result.x.item(0)

        self.throttle_front = throttle_f
        self.throttle_rear = throttle_r
        self.throttle_thrust = throttle_t
        return self.delta

# objective function to be minimized
def wrench_objective_fun(x, quadplane_model, wrench_des, state):
    quadplane_model._state = state
    quadplane_model._update_velocity_data()
    delta = MsgDelta(elevator=x)
    wrench_actual = quadplane_model._forces_moments(delta)
    J = np.linalg.norm(wrench_des - wrench_actual)**2.0
    return J

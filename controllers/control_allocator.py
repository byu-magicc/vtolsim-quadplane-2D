"""
    - Last Update:
        1/24/2025  - RWB
"""
import numpy as np 
from scipy.optimize import minimize
from models.quadplane_dynamics import QuadplaneDynamics
from message_types.msg_delta import MsgDelta
from message_types.msg_state import MsgState
from tools.saturate import saturate

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
        x0 = 0. # initial guess for elevator
        # define inequality constraints
        cons = ([
            {'type': 'ineq', #>0
                'fun': lambda x: np.array([# elevator constraint
                    np.radians(45)-x[0], # theta <= thetabar
                    x[0]+np.radians(45), # theta >= -thetabar
                    ]),
                # 'jac': lambda x: np.array([
                #     [-1.],
                #     [1.],
                #     ])
            }
        ])
        result = minimize(wrench_objective_fun1, 
                          x0, 
                          method='SLSQP', 
                          args = (self.quadplane_model, wrench_des, state),
                          constraints=cons, 
                          options={'ftol': 1e-10, 'disp': True, 'maxiter': 1000})
        self.delta.elevator = saturate(result.x.item(0), -np.radians(45), np.radians(45))
        x0 = np.array([
            0.5, # initial guess for throttle_front
            0.5, # initial guess for throttle_rear
            0.5, # initial guess for throttle_thrust
        ])
        # define inequality constraints
        cons = ([
            {'type': 'ineq', #>0
                'fun': lambda x: np.array([# elevator constraint
                    x[0],
                    x[1],
                    x[2],
                    1.-x[0],
                    1.-x[1],
                    1.-x[2],
                    ]),
                'jac': lambda x: np.array([
                    [1., 0., 0.],
                    [0., 1., 0.],
                    [0., 0., 1.],
                    [-1., 0., 0.],
                    [0., -1., 0.],
                    [0., 0., -1.],
                    ])
            }
        ])
        result = minimize(wrench_objective_fun2, 
                          x0, 
                          method='SLSQP', 
                          args = (self.quadplane_model, wrench_des, 
                                  state, self.delta.elevator),
                          constraints=cons, 
                          options={'ftol': 1e-10, 'disp': True, 'maxiter': 1000})
        self.delta.throttle_front = saturate(result.x.item(0), 0., 1.)
        self.delta.throttle_rear = saturate(result.x.item(1), 0., 1.)
        self.delta.throttle_thrust = saturate(result.x.item(2), 0., 1.)
        return self.delta

# objective functions to be minimized
def wrench_objective_fun1(x, quadplane_model, wrench_des, state):
    quadplane_model._set_internal_state(state)
    quadplane_model._update_velocity_data()
    delta = MsgDelta(elevator=x.item(0))
    wrench_actual = quadplane_model._forces_moments(delta)
    J = np.linalg.norm(wrench_des - wrench_actual)**2.0
    return J

def wrench_objective_fun2(x, quadplane_model, wrench_des, state, elevator):
    quadplane_model._set_internal_state(state)
    quadplane_model._update_velocity_data()
    delta = MsgDelta(elevator=elevator, 
                     throttle_front=x.item(0),
                     throttle_rear=x.item(1),
                     throttle_thrust=x.item(2),
                     )
    wrench_actual = quadplane_model._forces_moments(delta)
    J = np.linalg.norm(wrench_des - wrench_actual)**2.0
    return J

"""
compute_trim
    - Update history:
        12/29/2018 - RWB
        3/12/2024 - RWB
        1/20/2025 - RWB
"""
import numpy as np
from scipy.optimize import minimize
from message_types.msg_delta import MsgDelta
from models.quadplane_dynamics import QuadplaneDynamics


def compute_trim(quadplane: QuadplaneDynamics, 
                 Va: float, 
                 )->tuple[np.ndarray, MsgDelta]:
    # define initial state and input
    state0 = np.array([
        [quadplane._state.item(0)],  # pn [0]
        [quadplane._state.item(1)],  # pd [1]
        [Va],  # u [2] 
        [0.],  # w [3]
        [0.],  # theta0 [4]
        [0.],  # q [5]
        ])
    delta0 = np.array([[0.],  # elevator [6]
                       [0.5],  # throttle_front [7]
                       [0.5],  # throttle_rear [8]
                       [0.0],  # throttle_thrust [9]
                       ])
    x0 = np.concatenate((state0, delta0), axis=0)
    # define equality constraints
    cons = ([
        {'type': 'eq',
             'fun': lambda x: np.array([
                x[2]**2 + x[3]**2 - Va**2,  # magnitude of velocity is Va
                x[5], # pitch rate should be zero
                (x[7]-x[8]), # front and rear rotors are equal
                ]),
            #  'jac': lambda x: np.array([
            #     [0., 0., 2*x[2], 2*x[3], 0., 0., 0., 0., 0., 0.],
            #     [0., 0., 0., 0., 0., 1., 0., 0., 0., 0.],
            #     [0., 0., 0., 0., 0., 0., 0., 1., -1., 0.],
            #     ])
        }, 
        {'type': 'ineq', #>0
            'fun': lambda x: np.array([
                x[7], #3 motor inputs between 0 and 1
                x[8],
                x[9],
                (-x[7]+1.0),
                (-x[8]+1.0),
                (-x[9]+1.0),
                np.radians(25)-x[4], # theta <= thetabar
                x[4]+np.radians(25), # theta >= -thetabar
                ]),
            'jac': lambda x: np.array([
                [0., 0., 0., 0., 0., 0., 0., 1., 0., 0.],
                [0., 0., 0., 0., 0., 0., 0., 0., 1., 0.],
                [0., 0., 0., 0., 0., 0., 0., 0., 0., 1.],
                [0., 0., 0., 0., 0., 0., 0., -1., 0., 0.],
                [0., 0., 0., 0., 0., 0., 0., 0., -1., 0.],
                [0., 0., 0., 0., 0., 0., 0., 0., 0., -1.],
                [0., 0., 0., 0., -1., 0., 0., 0., 0., 0.],
                [0., 0., 0., 0., 1., 0., 0., 0., 0., 0.],
                ])
        }
            ])
    # solve the minimization problem to find the trim states and inputs

    result = minimize(trim_objective_fun, x0.flatten(), 
                      method='SLSQP', 
                      args = (quadplane, Va),
                      constraints=cons, 
                      options={'ftol': 1e-10, 'disp': True, 'maxiter': 1000})
    # result = minimize(trim_objective_fun, x0.flatten(), 
    #                   method='SLSQP', 
    #                   args = (quadplane, Va),
    #                   options={'ftol': 1e-10, 'disp': True, 'maxiter': 1000})    
    J = trim_objective_fun(result.x, quadplane, Va)
    # extract trim state and input and return
    trim_state = np.array([result.x[0:6]]).T
    trim_state[0,0] =  quadplane._state.item(0)  # pn [0]
    trim_state[1,0] = quadplane._state.item(1)  # pd [1]
    trim_input = np.array([result.x[6:10]]).T
    print('trim_state=', trim_state.T)
    print('trim_input=', trim_input.T)
    print('J_final=', J)

    trim_delta = MsgDelta()
    trim_delta.from_array(trim_input)
    return trim_state, trim_delta

# objective function to be minimized
def trim_objective_fun(x, quadplane, Va):
    state = np.array([x[0:6]]).T
    delta = MsgDelta()
    delta.elevator = x.item(6)
    delta.throttle_front = x.item(7)
    delta.throttle_rear = x.item(8)
    delta.throttle_thrust = x.item(9)
    xdot = np.array([[999., 999., 0., 0., 0., 0.]]).T
    quadplane._state = state
    quadplane._update_velocity_data()
    forces_moments = quadplane._forces_moments(delta)
    f = quadplane._f(state, forces_moments)
    tmp = xdot - f
    lam = 10.0
    J = np.linalg.norm(tmp[2:6])**2.0
    J += lam * delta.throttle_front**2
    J += lam * delta.throttle_rear**2
    #J += lam * delta.throttle_thrust**2
    #print('J=', J)
    return J


def compute_ss_model(vtol, trim_state, trim_input):
    A = df_dx(vtol, trim_state, trim_input)
    B = df_du(vtol, trim_state, trim_input)
    return A, B


def df_dx(quadplane, x, delta):
    '''take partial of f with respect to x'''
    eps = 0.01  # deviation
    A = np.zeros((6,6))  # Jacobian of f wrt x
    quadplane._state = x
    quadplane._update_velocity_data()
    forces_moments = quadplane._forces_moments(delta)
    f = quadplane._f(x, forces_moments)
    for i in range(0, 5):
        x_eps = np.copy(x)
        x_eps[i][0] += eps
        quadplane._state = x_eps
        quadplane._update_velocity_data()
        forces_moments = quadplane._forces_moments(delta)
        f_eps = quadplane._f(x_eps, forces_moments)
        df = (f_eps - f) / eps
        A[:,i] = df[:,0]
    return A

def df_du(quadplane, x, delta):
    '''take partial of f with respect to delta'''
    eps = 0.01  # deviation
    B = np.zeros((6, 4))  # Jacobian of f wrt u
    quadplane._state = x
    quadplane._update_velocity_data()
    forces_moments = quadplane._forces_moments(delta)
    f = quadplane._f(x, forces_moments)
    for i in range(0, 4):
        delta_eps = delta.to_array()
        delta_eps[i, 0] += eps
        delta_eps_ = MsgDelta()
        delta_eps_.from_array(delta_eps)
        forces_moments_eps = quadplane._forces_moments(delta_eps_)
        f_eps = quadplane._f(x, forces_moments_eps)
        df = (f_eps - f) / eps
        B[:,i] = df[:,0]
    return B

def print_ss_model(filename, A, B, Va, trim_state, trim_input):
    '''write state space model to file'''
    n = B.shape[0]
    m = B.shape[1]
    input = trim_input.to_array()
    # open file for writting
    file = open('models/' + filename, 'w')
    file.write('import numpy as np\n')
    # write the airspeed
    file.write('Va = %f\n' % Va)
    # write the trim state
    file.write('trim_state = np.array([[')
    for i in range(0,n):
        file.write('%f, ' % trim_state.item(i))  
    file.write(']]).T\n')      
    # write the trim input
    file.write('trim_input = np.array([[')
    for i in range(0,m):
        file.write('%f, ' % input.item(i))  
    file.write(']]).T\n')      
    # write A
    file.write('A = np.array([\n')
    for i in range(0,n):
        file.write('[')
        for j in range(0,n-1):
            file.write('%f, ' % A[i,j])
        file.write('%f],\n' % A[i,n-1])
    file.write('])\n')      
    # write B
    file.write('B = np.array([')
    for i in range(0,n):
        file.write('[')
        for j in range(0,m-1):
            file.write('%f, ' % B[i,j])
        file.write('%f],\n' % B[i,m-1])
    file.write('])\n')      
    # close file
    file.close()

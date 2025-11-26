import numpy as np
import parameters.control_parameters_quadcopter as AP
import parameters.anaconda_parameters as CONDA
from tools.old.rotations import euler_to_rotation, rotation_to_euler
from tools.wrap import wrap
from controllers.pid_control import PIDControl
from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta
from message_types.msg_autopilot import MsgAutopilot_Quadrotor
from tools.saturate import saturate
from scipy.optimize import minimize


#creates the autopilot
class Autopilot:

    def __init__(self, ts_control: float):
        #creates the pitch controller
        self.pitch_ctrl = PIDControl(kp=AP.pitch_kp,
                                     ki = 0,
                                     kd=AP.pitch_kd,
                                     limit=AP.xy_torque_limit)
        
        #creates the north controller
        self.north_ctrl = PIDControl(kp=AP.north_kp,
                                     ki=AP.north_ki,
                                     kd=AP.north_kd,
                                     limit=AP.north_max)
        
        #creates the down (altitude) controller
        self.down_ctrl = PIDControl(kp=AP.down_kp,
                                    ki=AP.down_ki,
                                    kd=AP.down_kd,
                                    limit=AP.down_max)
        
        #saves the commanded state
        self.commanded_state = MsgState()

    #creates the update function
    def update(self, 
               command: MsgAutopilot_Quadrotor,
               state: MsgState):
        
        #creates the rotation
        R = state.R
        #get the current euler angles
        phi, theta, psi = rotation_to_euler(R=R)
        
        #gets the omegas from the state
        omegas = state.omega
        #gets thie q
        q = omegas.item(1)

        #gets the commands from the autopilot message
        pos_cmd = command.position_cmd
        vel_cmd = command.velocity_cmd
        accel_cmd = command.accel_cmd

        #creates the autopilot for the north
        u_n = self.north_ctrl.update_with_ff(y_ref=pos_cmd.item(0),#north position command
                                             y=state.pos_2D.item(0),#north actual position
                                             y_ref_dot=vel_cmd.item(0),#north dot velocity command
                                             feedforward=accel_cmd.item(0))#north double dot accel command
        
        #creates the autopilot for the down position
        u_d = self.down_ctrl.update_with_ff(y_ref=pos_cmd.item(2),#down position command
                                            y=state.pos_2D.item(2),#down actual position
                                            y_ref_dot=vel_cmd.item(2),#down dot velocity  command
                                            feedforward=accel_cmd.item(2))#down double dot accel command
        

        #converts to the desired thrust
        thrust_total_desired = AP.mass * (AP.gravity - u_d)

        #creates the theta command
        theta_cmd = saturate(-u_n/AP.gravity, -AP.pitch_angle_limit, AP.pitch_angle_limit)
        
        #gets the torque desired command from the theta command
        torque_y_desired = self.pitch_ctrl.update_with_rate(y_ref=theta_cmd, #the theta commanded position
                                                            y=theta, #the actual theta position of the aircraft
                                                            ydot=q) #the pitch rate of the aircraft
        
        #gets the individual thrusts desired
        individualThrusts = CONDA.individualThrustUnmixer @ np.array([[thrust_total_desired],
                                                                   [torque_y_desired]])

        #gets the front and rear thrusts desired
        thrust_front_desired = individualThrusts.item(0)
        thrust_rear_desired = individualThrusts.item(1)

        #gets the airspeed through the front and rear props
        Va_front = -state.v_air.item(1)
        Va_rear = -state.v_air.item(1)

        #gets the delta front
        delta_front = self.invert_motor(Thrust_des=thrust_front_desired,
                                        Va=Va_front)
        
        #gets the delta for the rear
        delta_rear = self.invert_motor(Thrust_des=thrust_rear_desired,
                                       Va=Va_rear)
        
        #creates the return delta message
        deltaMessage = MsgDelta(elevator=0.0,
                                throttle_front=delta_front,
                                throttle_rear=delta_rear,
                                throttle_thrust=0.0)
        

        #updates the commanded state
        self.commanded_state.pos_2D = command.position_cmd
        self.commanded_state.vel_2D = command.velocity_cmd
        self.commanded_state.R = R

        #returns the delta message and the commanded state
        return deltaMessage, self.commanded_state


    #defines the function to get the delta command from a desired thrust for the rotor
    def invert_motor(self,
                     Thrust_des: float, #the desired thrust for the motor
                     Va: float): #the airspeed through the front of the propepper

        #defines the initial guess for the motor delta
        delta_init = 0.5

        #defines all of the constraints for everything
        cons = ([
            {'type': 'ineq', #>0
                'fun': lambda delta: np.array([# elevator constraint
                    1.-delta[0], # delta <=1 
                    delta[0], # delta>=0
                    ]),
                    'jac': lambda delta: np.array([
                    [-1.],
                    [1.],
                    ])
                }
            ])

        #gets the delta result
        result = minimize(self.objective_motor,
                          delta_init,
                          method='SLSQP',
                          args = (Thrust_des, Va),
                          constraints=cons,
                          options={'ftol': 1e-10, 'disp': True, 'maxiter': 1000, 'iprint': 0})
        
        #gets the delta from the result
        delta = saturate(result.x.item(0), 0.0, 1.0)
        #returns the delta
        return delta
    #creates the objective function for the motor
    def objective_motor(self, delta: float, Thrust_des: float, Va: float):

        #gets the motor thrust from the defined function below
        Thrust_motor = self.motor_thrust(Va=Va,
                                         delta_t = delta)
        
        #returns the squared error between the thrust desired and thrust motor
        J = (Thrust_des - Thrust_motor)**2
        #returns the oerror objective
        return J


    def motor_thrust(self, Va: float, delta_t: float)->float:
        C_Q0 = CONDA.C_Q0
        C_Q1 = CONDA.C_Q1
        C_T0 = CONDA.C_T0
        C_Q2 = CONDA.C_Q2
        C_T1 = CONDA.C_T1
        C_T2 = CONDA.C_T2
        D_prop = CONDA.D_prop
        KQ = CONDA.KQ
        R_motor = CONDA.R_motor
        i0 = CONDA.i0
        rho = CONDA.rho
        #gets the voltage in, based on the delta_t
        V_in = CONDA.V_max * delta_t
        # Quadratic formula to solve for motor speed
        a = C_Q0 * rho * np.power(D_prop, 5)/((2.*np.pi)**2)
        b = (C_Q1 * rho * np.power(D_prop, 4)/ (2.*np.pi)) * Va + KQ**2/R_motor
        c = C_Q2 * rho * np.power(D_prop, 3)* Va**2 - (KQ / R_motor) * V_in + KQ * i0        
        # Consider only positive root
        Omega_p = (-b + np.sqrt(b**2 - 4*a*c)) / (2.*a)
        aa = C_T0 * rho * np.power(D_prop,4) / (4. * np.pi**2)
        bb = C_T1 * rho * np.power(D_prop,3) * Va/(2 * np.pi)
        cc = C_T2 * rho * D_prop**2 * Va**2
        T_p = (aa) * Omega_p**2 + (bb) * Omega_p + (cc)
        return T_p
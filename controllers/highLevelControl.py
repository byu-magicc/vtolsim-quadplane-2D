from message_types.msg_state import MsgState
from message_types.msg_trajectory import MsgTrajectory
from message_types.msg_integrator import MsgIntegrator

import parameters.simulation_parameters as SIM
import parameters.control_allocation_parameters as CAP
import parameters.anaconda_parameters as CONDA
import parameters.highLevelControl_parameters as HLC

from rrt_mavsim.message_types.msg_plane import MsgPlane
from rrt_mavsim.tools.plane_projections_2 import map_3D_to_2D

from controllers.feedforwardControl import feedForwardControl
from controllers.pitch_optimization import PitchOptimization
from tools.rotations import theta_to_rotation_2D
import numpy as np


class highLevelControl:


    def __init__(self,
                 state: MsgState,
                 plane: MsgPlane,
                 theta_0: float = 0.0,
                 theta_dot_0: float = 0.0,
                 Ts: float = SIM.ts_simulation,
                 pitchControl_riseTime: float = CAP.pitchControl_riseTime,
                 pitchControl_zeta: float = CAP.pitchControl_zeta):

        self.Ts = Ts
        self.pitchControl_riseTime = pitchControl_riseTime
        self.pitchControl_zeta = pitchControl_zeta

        self.plane = plane

        #from this, we get the natural frequency
        pitchControl_naturalFrequency = np.pi / (2.0*pitchControl_riseTime*np.sqrt(1.0 - (pitchControl_zeta**2)))

        #creates the three variables for the main PD transfer function here
        b0 = CONDA.Jy
        a1 = 0.0
        a0 = 0.0

        #gets the kp and the kd variables
        kp_pitch = (pitchControl_naturalFrequency**2 - a0)/(b0)
        kd_pitch = (2*pitchControl_zeta*pitchControl_naturalFrequency - a1)/(b0)


        #creates the pitch controller
        self.pitchController = feedForwardControl(kp=kp_pitch,
                                                  kd=kd_pitch,
                                                  theta_0=theta_0,
                                                  theta_dot_0=theta_dot_0,
                                                  Ts=SIM.ts_simulation,
                                                  Jy=CONDA.Jy,
                                                  u_max=CAP.tau_max,
                                                  sigma=0.05)

        #creates the pitch optimizer, which obtains the desired pitch
        self.pitchOptimizer = PitchOptimization(state=state,
                                                Ts=Ts)

        #creates the integrator for just the position error (velocity error is not super important)
        self.integrator_posError = np.zeros((2,1))

        #sets the max error derivative for the integrator to integrate for antiwindup purposes
        self.max_error_derivative_antiwindup = HLC.max_error_derivative_antiwindup

        #the delayed by 1 sample of the error
        self.pos_error_d1 = np.zeros((2,1))

        self.integrator_msg = MsgIntegrator()

    #Arguments:
    #1. trajectory reference
    #2. state of aircraft
    #Returns:
    #1. Force Desired Body Frame
    #2. Moment Desired Body Frame
    def update(self,
               trajectory_ref: MsgTrajectory,
               state: MsgState):

        #gets the position
        position = state.pos_2D
        #and the velocity
        velocity = state.vel_2D

        #gets the position error
        position_error = trajectory_ref.pos - position
        #the velocity error
        velocity_error = trajectory_ref.vel - velocity

        #updates the integral
        self.update_integral(pos_error=position_error,
                             vel_error=velocity_error)

        e_up_2D = map_3D_to_2D(vec_3D=CONDA.e_up_3D,
                               plane=self.plane)

        #gets the error state vector
        errorState = np.concatenate((position_error, velocity_error), axis=0)

        #TODO get rid of these three. They're just for testing right now
        accelTerm_accel = trajectory_ref.accel

        #TODO get rid of these three. They're just for testing right now
        accelTerm_accel = trajectory_ref.accel
        gravityTerm_accel = CONDA.gravity*e_up_2D
        errorStateTerm_accel = HLC.K @ errorState
        integratorTerm_accel = HLC.Ki @ self.integrator_msg.getIntegrator()

        accelTerm_Force = CONDA.mass * accelTerm_accel
        gravityTerm_Force = CONDA.mass * gravityTerm_accel
        errorStateTerm_Force = CONDA.mass * errorStateTerm_accel
        integratorTerm_Force = CONDA.mass * integratorTerm_accel


        #given these terms, we create the simple trajectory following control law
        #gets the desired force on the aircraft in the inertial frame.
        # Equal to mass times the desired acceleration due to the trajectory B-Spline
        #plus gravity (negated because of the positive d vector pointing down, and we want a force to oppose that)
        #plus State-Feedback matrix K times error state 
        #TODO make gravity generalized-planable
        accel_des_i = trajectory_ref.accel + CONDA.gravity*e_up_2D + HLC.K @ errorState + HLC.Ki @ self.integrator_msg.getIntegrator()
        F_des_i = CONDA.mass * accel_des_i

        #given the Force desired, we call the pitch optimization function to get the optimal theta
        theta_ref = self.pitchOptimizer.update(state=state,
                                               state_ref=trajectory_ref,
                                               F_des_i=F_des_i)
        
        #gets the actual theta
        theta = state.theta
        q = state.q

        #with the theta desired, we call the pitch controller to get the M desired
        M_des_b = self.pitchController.update(state=theta,
                                    state_dot=q,
                                    state_ref=theta_ref)
        
        #gets the force desired in the body frame
        R_b2i = theta_to_rotation_2D(theta=theta)
        R_i2b = R_b2i.T
        F_des_b = R_i2b @ F_des_i

        #saves the current position error as delayed by 1
        self.pos_error_d1 = position_error

        #returns these two things to go to the low level control
        return F_des_b, M_des_b

    #defines the update integral function
    def update_integral(self,
                        pos_error: np.ndarray,
                        vel_error: np.ndarray):
        

        #gets the number of bins to operate on
        numVelSlots = np.size(vel_error)

        for i in range(numVelSlots):
            #gets the velocity error delayed by 1
            vel_error_temp = vel_error.item(i)
            #if the value of the velocity error is less than the threshold, we integrate
            if np.abs(vel_error_temp) < self.max_error_derivative_antiwindup:
                #gets the position error at this point
                pos_error_temp = pos_error.item(i)
                #gets the position error delayed by 1
                pos_error_temp_d1 = self.pos_error_d1.item(i)

                #adds to the integration variable
                self.integrator_posError[i,:] = self.integrator_posError[i,:] +\
                                                (self.Ts/2.0) * (pos_error_temp + pos_error_temp_d1)
            #otherwise, we pass and do nothing.
            else:
                pass
            potato = 0

        self.integrator_msg.setIntegrator(integratorValue=self.integrator_posError)


    def getIntegrator(self):
        return self.integrator_msg

#this is the path following algorithm we need to make the low level controller operate correctly.

from message_types.msg_state import MsgState
from message_types.msg_trajectory import MsgTrajectory

import parameters.simulation_parameters as SIM
import parameters.control_allocation_parameters as CAP
import parameters.anaconda_parameters as CONDA
import parameters.highLevelControl_parameters as HLC

from controllers.feedforwardControl import feedForwardControl
from controllers.pitch_optimization import PitchOptimization
from tools.old.rotations import *

import numpy as np

e_down = np.array([[0.0],
                   [1.0]])


class highLevelControl:


    def __init__(self,
                 state: MsgState,
                 Ts: float = SIM.ts_simulation,
                 pitchControl_riseTime: float = CAP.pitchControl_riseTime,
                 pitchControl_zeta: float = CAP.pitchControl_zeta):

        self.Ts = Ts
        self.pitchControl_riseTime = pitchControl_riseTime
        self.pitchControl_zeta = pitchControl_zeta

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
                                                  Ts=SIM.ts_simulation,
                                                  Jy=CONDA.Jy,
                                                  u_max=CAP.tau_max,
                                                  sigma=0.05)
        

        #creates the pitch optimizer, which obtains the desired pitch
        self.pitchOptimizer = PitchOptimization(state=state,
                                                Ts=Ts)


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
        position_error = position - trajectory_ref.pos
        #the velocity error
        velocity_error = velocity - trajectory_ref.vel

        #gets the error state vector
        errorState = np.concatenate((position_error, velocity_error), axis=0)

        #TODO get rid of these three. They're just for testing right now
        accelTerm = CONDA.mass * trajectory_ref.accel
        gravityTerm = -CONDA.mass*CONDA.gravity*e_down
        errorStateTerm = -CONDA.mass*HLC.K @ errorState

        #given these terms, we create the simple trajectory following control law
        #gets the desired force on the aircraft in the inertial frame.
        # Equal to mass times the desired acceleration due to the trajectory B-Spline
        #plus gravity (negated because of the positive d vector pointing down, and we want a force to oppose that)
        #plus State-Feedback matrix K times error state 
        F_des_i = CONDA.mass * (trajectory_ref.accel - CONDA.gravity*e_down - HLC.K @ errorState)
        
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
        F_des_b = theta_to_rotation_2d(theta=theta) @ F_des_i

        #returns these two things to go to the low level control
        return F_des_b, M_des_b


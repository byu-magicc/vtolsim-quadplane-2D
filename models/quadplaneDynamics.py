
#the quadplane dynamics file

import numpy as np
import parameters.anaconda_parameters as CONDA
import parameters.simulation_parameters as SIM
from tools.rotations import *

from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta

from rrt_mavsim.message_types.msg_plane import MsgPlane
from rrt_mavsim.tools.plane_projections import *



#creates the Quadplane dynamics class
class QuadplaneDynamics:

    #creates the initialization function
    def __init__(self,
                 plane_msg: MsgPlane,
                 ts: float = SIM.ts_simulation,
                 pn0_3D: float=CONDA.pn0,
                 pd0_3D: float=CONDA.pd0,
                 pn_dot0_3D: float=CONDA.pn_dot0,
                 pd_dot0_3D: float=CONDA.pd_dot0,
                 theta0: float=CONDA.theta0,
                 q0: float=CONDA.q0):
        

        self.plane_msg = plane_msg
        self.ts = ts

        #gets the 3D position and Projects it onto the respective plane
        pos_3D_init = np.array([[pn0_3D],[0.0],[pd0_3D]])
        pos_2D_init = map_3D_to_2D_planeMsg(vec_3D=pos_3D_init,
                                            plane_msg=plane_msg)
        
        vel_3D_init = np.array([[pn_dot0_3D],[0.0],[pd_dot0_3D]])
        vel_2D_init = map_3D_to_2D_planeMsg(vec_3D=vel_3D_init,
                                            plane_msg=plane_msg)


        #creates the state array from the initialized 2D points
        self._state = np.concatenate((pos_2D_init, vel_2D_init, np.array([[theta0]]), np.array([[q0]])))
        

        #creates the true state
        self.true_state = MsgState()
        #updates the vleocity data, as in the wind and everything else 
        # (assume no wind for now)
        self._update_velocity_data()
        #initially calls the forces momemnts function
        self._forces_moments(delta=MsgDelta())

        potato = 0

    #the update velocity data function
    def _update_velocity_data(self,
                              wind: np.ndarray=np.zeros((4,1))):
        #gets the steady state wind from the input variable
        #gets the steady state wind
        steadyStateWind_inertial = wind[0:2]
        gust = wind[2:4]

        theta=self._state.item(4)

        #gets the Rotation from Body to inertial
        R_body2inertial = theta_to_rotation_2D(theta=theta)
        #and it's inverse
        R_inertial2body = R_body2inertial.T

        #gets the wind in the body frame
        wind_body = R_inertial2body @ steadyStateWind_inertial
        wind_body += gust
        wind_inertial = R_body2inertial @ wind_body
        self._wind = wind_inertial


        #gets the velocity in the inertial frame
        velocity_inertial_frame = self._state[2:4,0:1]
        #gets the air velocity with respect to the inertial frame
        self.v_air_inertial = velocity_inertial_frame - wind_inertial
        #gets the v air in the body frame
        self.v_air_body = R_inertial2body @ self.v_air_inertial

        #gets the airspeed magnitude
        self._Va = np.linalg.norm(self.v_air_body)
        ur = self.v_air_body.item(0)
        wr = self.v_air_body.item(1)
        # compute angle of attack
        if ur==0:
            self._alpha = -np.sign(wr)*np.pi/2.
        else:
            self._alpha = np.arctan2(-wr, ur)

        potato = 0

    #creates the forces and moments
    def _forces_moments(self, delta: MsgDelta)->np.ndarray:

        #gets the rotation from body to the world frame
        theta = self._state.item(4)
        #gets the rotation matrix from the body to the inertial
        R_body2Inertial_2D = theta_to_rotation_2D(theta=theta)
        #gets the rotation matrix from the inertial to the body
        R_inertial2Body_2D = np.transpose(R_body2Inertial_2D)

        #gets the pitch rate
        q = self._state.item(5)

        #gets the force of gravity in the inertial frame
        fg_inertial_3D = CONDA.mass * CONDA.e3_3D
        #gets it in the 2D frame
        fg_inertial_2D = map_3D_to_2D_planeMsg(vec_3D=fg_inertial_3D,
                                               plane_msg=self.plane_msg)
        #rotates it into the body frame
        fg_body_2D = R_inertial2Body_2D @ fg_inertial_2D

        #initializes the main forces fx and fz in the body frame.
        #we will add to this as the function progresses. This is the first term.
        fx_body = fg_body_2D.item(0)
        fz_body = fg_body_2D.item(1)

        #q_bar is the coefficient out front of all the aerodynamic forces and moments
        qbar = 0.5 * CONDA.rho * self._Va**2

        #gets the cosine and sine of alpha
        ca = np.cos(self._alpha)
        sa = np.sin(self._alpha)

        # nondimensionalize q
        if self._Va > 1:
            q_nondim = q * CONDA.c / (2 * self._Va)
        else:
            q_nondim = 0.0

        # compute Lift and Drag coefficients
        tmp1 = np.exp(-CONDA.M * (self._alpha - CONDA.alpha0))
        tmp2 = np.exp(CONDA.M * (self._alpha + CONDA.alpha0))
        sigma = (1.0 + tmp1 + tmp2) / ((1.0 + tmp1) * (1.0 + tmp2))
        CL = (1.0 - sigma) * (CONDA.C_L_0 + CONDA.C_L_alpha * self._alpha) \
             + sigma * 2 * np.sign(self._alpha) * sa**2 * ca
        CD = CONDA.C_D_p + ((CONDA.C_L_0 + CONDA.C_L_alpha * self._alpha)**2)/(np.pi * CONDA.e * CONDA.AR)
        # compute Lift and Drag Forces
        F_lift = qbar * CONDA.S_wing * (CL + CONDA.C_L_q * q_nondim + CONDA.C_L_delta_e * delta.elevator)
        F_drag = qbar * CONDA.S_wing * (CD + CONDA.C_D_q * q_nondim + CONDA.C_D_delta_e * delta.elevator)

        #adds the Lift and Drag to the components
        fx_body += -ca * F_drag + sa * F_lift
        fz_body += sa * F_drag + ca * F_lift

        #computes the pitching moment
        My = qbar * CONDA.S_wing * CONDA.c * (
            CONDA.C_m_0
            + CONDA.C_m_alpha * self._alpha
        )
        

        potato = 0
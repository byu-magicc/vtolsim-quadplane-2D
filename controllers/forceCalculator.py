from message_types.msg_delta import MsgDelta
import numpy as np
from tools.rotations import alphaToRotation, theta_to_rotation_2D
import parameters.anaconda_parameters as CONDA
from rrt_mavsim.message_types.msg_plane import MsgPlane
from rrt_mavsim.tools.plane_projections_2 import map_3D_to_2D, map_2D_to_3D


class forceCalculator:

    def __init__(self):

        pass



    def _forces_moments(self,
                        delta: MsgDelta,
                        plane: MsgPlane,
                        Va: float,
                        alpha: float,
                        state: np.ndarray):

        #gets the rotation matrixes
        theta = state.item(CONDA.theta_index)
        R_bodyToInertial = theta_to_rotation_2D(theta=theta)
        R_inertialToBody = R_bodyToInertial.T

        q = state.item(CONDA.q_index)

        #obtains the force of gravity in the inertial frame
        fg_inertial_3D = CONDA.mass * CONDA.gravity * CONDA.e3_3D
        #rotates it into the 2D frame
        fg_inertial_2D = map_3D_to_2D(vec_3D=fg_inertial_3D,
                                      plane=plane)
        #rotates it into the body frame
        fg_body_2D = R_inertialToBody @ fg_inertial_2D


        #initializes the fx and fy in the body frame with gravity first
        fn_body = fg_body_2D.item(0)
        fd_body = fg_body_2D.item(1)

        #gets the constant coefficients
        #TODO I need to make sure _Va is getting updated
        q_bar = 0.5*CONDA.rho*Va**2

        #gets the cosine and sine of alpha
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        # nondimensionalize q
        if Va > 1:
            q_nondim = q * CONDA.c / (2 * Va)
        else:
            q_nondim = 0.0

        # compute Lift and Drag coefficients
        tmp1 = np.exp(-CONDA.M * (alpha - CONDA.alpha0))
        tmp2 = np.exp(CONDA.M * (alpha + CONDA.alpha0))
        sigma = (1.0 + tmp1 + tmp2) / ((1.0 + tmp1) * (1.0 + tmp2))
        CL = (1.0 - sigma) * (CONDA.C_L_0 + CONDA.C_L_alpha * alpha) \
             + sigma * 2 * np.sign(alpha) * sa**2 * ca
        CD = CONDA.C_D_p + ((CONDA.C_L_0 + CONDA.C_L_alpha * alpha)**2)/(np.pi * CONDA.e * CONDA.AR)

        # compute Lift and Drag Forces
        F_lift = q_bar * CONDA.S_wing * (CL + CONDA.C_L_q * q_nondim + CONDA.C_L_delta_e * delta.elevator)
        #very important to add the absolute value to the delta_e portion
        F_drag = q_bar * CONDA.S_wing * (CD + CONDA.C_D_q * q_nondim + CONDA.C_D_delta_e * np.abs(delta.elevator))

        #gets the alpha rotation matrix
        R_alpha = alphaToRotation(alpha=alpha)

        #gets the body frame aerodynamic forces
        f_aero_body = R_alpha @ np.array([[F_drag],[F_lift]])
        fn_aero_body = f_aero_body.item(0)
        fd_aero_body = f_aero_body.item(1)

        #adds to the body frame forces
        fn_body += fn_aero_body
        fd_body += fd_aero_body

        #computes the pitching moment
        My = q_bar * CONDA.S_wing * CONDA.c * (
                CONDA.C_m_0
                + CONDA.C_m_alpha * alpha
                + CONDA.C_m_q * q_nondim
                + CONDA.C_m_delta_e * delta.elevator)

        #gets the three thrusts from the simplified (constant*delta) model
        Thrust_front = motor_thrust_torque_simplified(delta_t=delta.throttle_front)
        Thrust_rear = motor_thrust_torque_simplified(delta_t=delta.throttle_rear)
        Thrust_forward = motor_thrust_torque_simplified(delta_t=delta.throttle_thrust)

        #gets the thrust vector
        thrustVector = np.array([[Thrust_front],[Thrust_rear],[Thrust_forward]])
        #gets the body frame thrust forces
        f_thrust_body = CONDA.thrust_forces_mixing @ thrustVector
        fn_thrust_body = f_thrust_body.item(0)
        fd_thrust_body = f_thrust_body.item(1)

        fn_body += fn_thrust_body
        fd_body += fd_thrust_body
        
        M_thrust = CONDA.thrust_moments_mixing @ thrustVector
        
        #and then adds it
        My += M_thrust.item(0)

        forces_moments = np.array([[fn_body],[fd_body],[My]])

        
        return forces_moments



#creates the new motor thrust and torque function, which is an over simplification,
#but is better for our simpler modelling and figuring everything out
def motor_thrust_torque_simplified(delta_t: float):

    if delta_t > 1.0:
        delta_t = 1.0
    #returns the delta_t times the max thrust
    return delta_t*CONDA.MaxThrust

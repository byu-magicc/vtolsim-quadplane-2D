import numpy as np
from scipy.optimize import minimize

from trajectory import Trajectory
from bspline_trajectory_member import BSplineTrajectoryMember


class BSplineTrajectoryGenerator:

    def generate_trajectory(
            self,
            path_points,
            degree,
            segment_times=None):

        member_list = []

        if path_points.ndim > 1:
            for member_i in range(0, path_points.shape[0]):
                member_list.append(
                    BSplineTrajectoryMember(path_points[member_i, :], degree)
                )
        else:
            member_list.append(
                BSplineTrajectoryMember(path_points[:], degree)
            )


        return Trajectory(member_list)

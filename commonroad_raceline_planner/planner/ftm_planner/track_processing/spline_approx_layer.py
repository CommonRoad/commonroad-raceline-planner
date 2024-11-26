import copy
import warnings

from scipy.interpolate import splprep, splev
from scipy.optimize import fmin
from scipy.spatial.distance import euclidean
import math
import numpy as np

from commonroad_raceline_planner.planner.ftm_planner.track_processing.base_layer import (
    BaseRacetrackLayer,
)
from commonroad_raceline_planner.planner.ftm_planner.ftm_dto import (
    DtoFTM,
    DtoFTMFactory,
)
from commonroad_raceline_planner.planner.ftm_planner.trajectory_planning.side_of_line import (
    side_of_line,
)

from typing import List


class SplineApproxLayer(BaseRacetrackLayer):
    """
    Spline approximation layer
    """

    def spline_approximation(
        self,
        dto_racetrack: DtoFTM,
        dto_racetrack_interpolated: DtoFTM,
        k_reg: int = 3,
        s_reg: int = 10,
        stepsize_reg: float = 3.0,
        debug: bool = False,
    ) -> DtoFTM:

        # TODO: split into different methods

        original_track: DtoFTM = copy.deepcopy(dto_racetrack)
        interpol_track: DtoFTM = copy.deepcopy(dto_racetrack_interpolated)

        # close race track if not already done
        if not original_track.is_closed:
            original_track.close_racetrack()

        if not interpol_track.is_closed:
            interpol_track.close_racetrack()

        if not interpol_track.is_interpolated:
            warnings.warn("racetrack is not interpolated, continuing anyways")

        # --- Interpolated track
        # find B spline representation of the inserted path and smooth it in this process
        # (tck_cl: tuple (vector of knots, the B-spline coefficients, and the degree of the spline) t_glob=parameter value)
        tck_cl, t_glob_cl = splprep(
            x=[interpol_track.x_m, interpol_track.y_m], k=k_reg, s=s_reg, per=1
        )[:2]

        # --- Original track
        # calculate total length of smooth approximating spline based on euclidian distance with points at every 0.25m
        no_points_lencalc_cl = math.ceil(original_track.track_length) * 4
        path_smoothed_tmp = np.array(
            splev(np.linspace(start=0.0, stop=1.0, num=no_points_lencalc_cl), tck_cl)
        ).T
        len_path_smoothed_tmp = np.sum(
            np.sqrt(np.sum(np.power(np.diff(path_smoothed_tmp, axis=0), 2), axis=1))
        )

        # get smoothed path
        no_points_reg_cl = math.ceil(len_path_smoothed_tmp / stepsize_reg) + 1
        path_smoothed = np.array(
            splev(np.linspace(start=0.0, stop=1.0, num=no_points_reg_cl), tck_cl)
        ).T[:-1]

        # ------------------------------------------------------------------------------------------------------------------
        # PROCESS TRACK WIDTHS (AND BANKING ANGLE IF GIVEN) ----------------------------------------------------------------
        # ------------------------------------------------------------------------------------------------------------------

        # find the closest points on the B spline to input points
        dists_cl: np.ndarray = np.zeros(
            original_track.num_points
        )  # contains (min) distances between input points and spline
        closest_point_cl: np.ndarray = np.zeros(
            (original_track.num_points, 2)
        )  # contains the closest points on the spline
        closest_t_glob_cl: np.ndarray = np.zeros(
            original_track.num_points
        )  # containts the t_glob values for closest points
        t_glob_guess_cl: np.ndarray = (
            original_track.track_length_per_point / original_track.track_length
        )  # start guess for the minimization

        for i in range(original_track.num_points):
            # get t_glob value for the point on the B spline with a minimum distance to the input points
            closest_t_glob_cl[i] = fmin(
                self._distance_to_point_on_spline,
                x0=t_glob_guess_cl[i],
                args=(
                    tck_cl,
                    np.asarray([original_track.x_m[i], original_track.y_m[i]]),
                ),
                disp=False,
            )

            # evaluate B spline on the basis of t_glob to obtain the closest point
            closest_point_cl[i] = splev(closest_t_glob_cl[i], tck_cl)

            # save distance from closest point to input point
            dists_cl[i] = math.sqrt(
                math.pow(closest_point_cl[i, 0] - original_track.x_m[i], 2)
                + math.pow(closest_point_cl[i, 1] - original_track.y_m[i], 2)
            )

        # get side of smoothed track compared to the inserted track
        sides = np.zeros(original_track.num_points - 1)

        for i in range(original_track.num_points - 1):
            sides[i] = side_of_line(
                a=(original_track.x_m[i], original_track.y_m[i]),
                b=(original_track.x_m[i + 1], original_track.y_m[i + 1]),
                z=closest_point_cl[i],
            )

        sides_cl = np.hstack((sides, sides[0]))

        # --- calculate new race track sides
        # calculate new track widths on the basis of the new reference line, but not interpolated to new stepsize yet
        w_tr_right_new_cl = original_track.w_tr_right_m + sides_cl * dists_cl
        w_tr_left_new_cl = original_track.w_tr_left_m - sides_cl * dists_cl

        # interpolate track widths after smoothing (linear)
        w_tr_right_smoothed_cl = np.interp(
            np.linspace(0.0, 1.0, no_points_reg_cl),
            closest_t_glob_cl,
            w_tr_right_new_cl,
        )
        w_tr_left_smoothed_cl = np.interp(
            np.linspace(0.0, 1.0, no_points_reg_cl), closest_t_glob_cl, w_tr_left_new_cl
        )

        track_reg = np.column_stack(
            (path_smoothed, w_tr_right_smoothed_cl[:-1], w_tr_left_smoothed_cl[:-1])
        )

        spline_track = DtoFTMFactory().generate_from_centerline_and_bounds(
            race_track=original_track.original_track,
            x_m=track_reg[:, 0],
            y_m=track_reg[:, 1],
            w_tr_right_m=track_reg[:, 2],
            w_tr_left_m=track_reg[:, 3],
            is_closed=False,
            is_interpolated=True,
            is_spline_approximated=True,
        )

        spline_track.close_racetrack()
        return spline_track

    @staticmethod
    def _distance_to_point_on_spline(
        t_glob: np.ndarray, path: List[float], p: np.ndarray
    ) -> float:
        """
        Returns distance ot point on spline. Used as cost function for minimization
        :param t_glob: parameter values of the spline
        :param path: path of the spline
        :param p: point
        :return: distance to point on spline
        """
        s = splev(t_glob, path)
        s_conv = [s[0][0], s[1][0]]
        return euclidean(p, s_conv)

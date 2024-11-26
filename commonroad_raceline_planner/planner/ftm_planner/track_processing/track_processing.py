import numpy as np
import matplotlib.pyplot as plt
import math

from commonroad_raceline_planner.planner.ftm_planner.track_processing.width_inflation_layer import (
    WidthInflationLayer,
)

# CommonRoad
from commonroad_raceline_planner.planner.ftm_planner.ftm_dto import DtoFTM
from commonroad_raceline_planner.planner.ftm_planner.trajectory_planning.calc_splines import (
    calc_splines,
)
from commonroad_raceline_planner.planner.ftm_planner.trajectory_planning.check_normals_crossing import (
    check_normals_crossing,
)
from commonroad_raceline_planner.planner.ftm_planner.track_processing.lin_interpol_layer import (
    LinearInterpolationLayer,
)
from commonroad_raceline_planner.planner.ftm_planner.track_processing.spline_approx_layer import (
    SplineApproxLayer,
)


# typing
from typing import Tuple


def preprocess_track(
    race_track: DtoFTM,
    k_reg: int,
    s_reg: int,
    stepsize_prep: float,
    stepsize_reg: float,
    debug: bool = True,
    min_width: float = None,
    normal_crossing_horizon: int = 10,
) -> Tuple[DtoFTM, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """

    Inputs:
    race_track:               imported track [x_m, y_m, w_tr_right_m, w_tr_left_m]
    reg_smooth_opts:            parameters for the spline approximation
    stepsize_opts:              dict containing the stepsizes before spline approximation and after spline interpolation
    debug:                      boolean showing if debug messages should be printed
    min_width:                  [m] minimum enforced track width (None to deactivate)

    Outputs:
    reftrack_interp:            track after smoothing and interpolation [x_m, y_m, w_tr_right_m, w_tr_left_m]
    normvec_normalized_interp:  normalized normal vectors on the reference line [x_m, y_m]
    a_interp:                   LES coefficients when calculating the splines
    coeffs_x_interp:            spline coefficients of the x-component
    coeffs_y_interp:            spline coefficients of the y-component
    """

    # ------------------------------------------------------------------------------------------------------------------
    # INTERPOLATE REFTRACK AND CALCULATE INITIAL SPLINES ---------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # close race track if not already done
    race_track.close_racetrack()

    # liner interpolation
    interpolated_track: (
        DtoFTM
    ) = LinearInterpolationLayer().linear_interpolate_racetrack(
        dto_racetrack=race_track,
        interpol_stepsize=stepsize_prep,
        return_new_instance=True,
    )

    spline_track: DtoFTM = SplineApproxLayer().spline_approximation(
        dto_racetrack=race_track,
        dto_racetrack_interpolated=interpolated_track,
        k_reg=k_reg,
        s_reg=s_reg,
        stepsize_reg=stepsize_reg,
        debug=debug,
    )

    # compute normals
    coeffs_x_interp, coeffs_y_interp, a_interp, normvec_normalized_interp = (
        compute_normals_and_check_crosing(
            race_track=spline_track, normal_crossing_horizon=normal_crossing_horizon
        )
    )

    # inflate track
    if min_width is not None:
        inflated_track: DtoFTM = WidthInflationLayer().inflate_width(
            dto_racetrack=spline_track,
            mininmum_track_width=min_width,
            return_new_instance=False,
        )
    else:
        inflated_track: DtoFTM = spline_track

    return (
        inflated_track,
        normvec_normalized_interp,
        a_interp,
        coeffs_x_interp,
        coeffs_y_interp,
    )


def compute_normals_and_check_crosing(
    race_track: DtoFTM, normal_crossing_horizon: int = 10
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Checks if normals crossing. Assumes a spline interpolation to be done first!
    :param race_track: dto racetrack object
    :return: Tuple[x_spline_coeff, y_spline_coeff, linear_system_matrix, normalized_normal_vectors]
    :raise: IOError if splines are crossing
    """
    # Check if normals are crossing
    coeffs_x_interp, coeffs_y_interp, a_interp, normvec_normalized_interp = (
        calc_splines(path=race_track.to_2d_np_array())
    )

    # TODO: may have to remove last point if closed?
    race_track.open_racetrack()
    normals_crossing = check_normals_crossing(
        track=race_track.to_4d_np_array(),
        normvec_normalized=normvec_normalized_interp,
        horizon=normal_crossing_horizon,
    )

    if normals_crossing:
        bound_1_tmp = (
            race_track.to_2d_np_array()
            + normvec_normalized_interp
            * np.expand_dims(race_track.to_2d_np_array(), axis=1)
        )
        bound_2_tmp = (
            race_track.to_2d_np_array()
            - normvec_normalized_interp
            * np.expand_dims(race_track.to_2d_np_array(), axis=1)
        )

        plt.figure()

        plt.plot(race_track.w_tr_right_m, race_track.w_tr_left_m, "k-")
        for i in range(bound_1_tmp.shape[0]):
            temp = np.vstack((bound_1_tmp[i], bound_2_tmp[i]))
            plt.plot(temp[:, 0], temp[:, 1], "r-", linewidth=0.7)

        plt.grid()
        ax = plt.gca()
        ax.set_aspect("equal", "datalim")
        plt.xlabel("east in m")
        plt.ylabel("north in m")
        plt.title("Error: at least one pair of normals is crossed!")

        plt.show()

        raise IOError(
            "At least two spline normals are crossed, check input or increase smoothing factor!"
        )

    else:
        return (coeffs_x_interp, coeffs_y_interp, a_interp, normvec_normalized_interp)


def calc_min_bound_dists(
    trajectory: np.ndarray,
    bound1: np.ndarray,
    bound2: np.ndarray,
    length_veh: float,
    width_veh: float,
) -> np.ndarray:
    """
    Created by:
    Alexander Heilmeier

    Documentation:
    Calculate minimum distance between vehicle and track boundaries for every trajectory point. Vehicle dimensions are
    taken into account for this calculation. Vehicle orientation is assumed to be the same as the heading of the
    trajectory.

    Inputs:
    trajectory:     array containing the trajectory information. Required are x, y, psi for every point
    bound1/2:       arrays containing the track boundaries [x, y]
    length_veh:     real vehicle length in m
    width_veh:      real vehicle width in m

    Outputs:
    min_dists:      minimum distance to boundaries for every trajectory point
    """

    # ------------------------------------------------------------------------------------------------------------------
    # CALCULATE MINIMUM DISTANCES --------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    bounds = np.vstack((bound1, bound2))

    # calculate static vehicle edge positions [x, y] for psi = 0
    fl = np.array([-width_veh / 2, length_veh / 2])
    fr = np.array([width_veh / 2, length_veh / 2])
    rl = np.array([-width_veh / 2, -length_veh / 2])
    rr = np.array([width_veh / 2, -length_veh / 2])

    # loop through all the raceline points
    min_dists = np.zeros(trajectory.shape[0])
    mat_rot = np.zeros((2, 2))

    for i in range(trajectory.shape[0]):
        mat_rot[0, 0] = math.cos(trajectory[i, 3])
        mat_rot[0, 1] = -math.sin(trajectory[i, 3])
        mat_rot[1, 0] = math.sin(trajectory[i, 3])
        mat_rot[1, 1] = math.cos(trajectory[i, 3])

        # calculate positions of vehicle edges
        fl_ = trajectory[i, 1:3] + np.matmul(mat_rot, fl)
        fr_ = trajectory[i, 1:3] + np.matmul(mat_rot, fr)
        rl_ = trajectory[i, 1:3] + np.matmul(mat_rot, rl)
        rr_ = trajectory[i, 1:3] + np.matmul(mat_rot, rr)

        # get minimum distances of vehicle edges to any boundary point
        fl__mindist = np.sqrt(
            np.power(bounds[:, 0] - fl_[0], 2) + np.power(bounds[:, 1] - fl_[1], 2)
        )
        fr__mindist = np.sqrt(
            np.power(bounds[:, 0] - fr_[0], 2) + np.power(bounds[:, 1] - fr_[1], 2)
        )
        rl__mindist = np.sqrt(
            np.power(bounds[:, 0] - rl_[0], 2) + np.power(bounds[:, 1] - rl_[1], 2)
        )
        rr__mindist = np.sqrt(
            np.power(bounds[:, 0] - rr_[0], 2) + np.power(bounds[:, 1] - rr_[1], 2)
        )

        # save overall minimum distance of current vehicle position
        min_dists[i] = np.amin((fl__mindist, fr__mindist, rl__mindist, rr__mindist))

    return min_dists


def interp_track(reftrack: np.ndarray, stepsize_approx: float = 1.0) -> np.ndarray:
    """
    Created by:
    Alexander Heilmeier

    Documentation:
    Use linear interpolation between track points to create new points with equal distances.

    Inputs:
    reftrack:           array containing the track information that shell be interpolated [x, y, w_tr_right, w_tr_left].
    stepsize_approx:    desired stepsize for the interpolation

    Outputs:
    reftrack_interp:    interpolated reference track (unclosed)
    """

    # ------------------------------------------------------------------------------------------------------------------
    # FUNCTION BODY ----------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    reftrack_cl = np.vstack((reftrack, reftrack[0]))

    # calculate element lengths (euclidian distance)
    el_lenghts = np.sqrt(
        np.sum(np.power(np.diff(reftrack_cl[:, :2], axis=0), 2), axis=1)
    )

    # sum up total distance (from start) to every element
    dists_cum = np.cumsum(el_lenghts)
    dists_cum = np.insert(dists_cum, 0, 0.0)

    # calculate desired lenghts depending on specified stepsize (+1 because last element is included)
    no_points_interp = math.ceil(dists_cum[-1] / stepsize_approx) + 1
    dists_interp = np.linspace(0.0, dists_cum[-1], no_points_interp)

    # interpolate closed track points
    reftrack_interp_cl = np.zeros((no_points_interp, 4))
    reftrack_interp_cl[:, 0] = np.interp(dists_interp, dists_cum, reftrack_cl[:, 0])
    reftrack_interp_cl[:, 1] = np.interp(dists_interp, dists_cum, reftrack_cl[:, 1])
    reftrack_interp_cl[:, 2] = np.interp(dists_interp, dists_cum, reftrack_cl[:, 2])
    reftrack_interp_cl[:, 3] = np.interp(dists_interp, dists_cum, reftrack_cl[:, 3])

    # remove closed points
    reftrack_interp = reftrack_interp_cl[:-1]

    return reftrack_interp

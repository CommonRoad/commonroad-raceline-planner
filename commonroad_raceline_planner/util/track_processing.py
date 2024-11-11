import numpy as np
import sys
import matplotlib.pyplot as plt
import math

# CommonRoad
from commonroad_raceline_planner.ractetrack import RaceTrack
from commonroad_raceline_planner.smoothing.spline_approximation import spline_approximation
from commonroad_raceline_planner.util.trajectory_planning_helpers.calc_splines import calc_splines
from commonroad_raceline_planner.util.trajectory_planning_helpers.check_normals_crossing import check_normals_crossing


def preprocess_track(race_track: RaceTrack,
                     reg_smooth_opts: dict,
                     stepsize_opts: dict,
                     debug: bool = True,
                     min_width: float = None) -> tuple:
    """
    Created by:
    Alexander Heilmeier

    Documentation:
    This function prepares the inserted reference track for optimization.

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

    # smoothing and interpolating reference track
    reftrack_interp = spline_approximation(track=race_track,
                                           k_reg=reg_smooth_opts["k_reg"],
                                           s_reg=reg_smooth_opts["s_reg"],
                                           stepsize_prep=stepsize_opts["stepsize_prep"],
                                           stepsize_reg=stepsize_opts["stepsize_reg"],
                                           debug=debug)

    # calculate splines
    refpath_interp_cl = np.vstack((reftrack_interp[:, :2], reftrack_interp[0, :2]))

    coeffs_x_interp, coeffs_y_interp, a_interp, normvec_normalized_interp = calc_splines(path=refpath_interp_cl)

    # ------------------------------------------------------------------------------------------------------------------
    # CHECK SPLINE NORMALS FOR CROSSING POINTS -------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    normals_crossing = check_normals_crossing(track=reftrack_interp,
                                                        normvec_normalized=normvec_normalized_interp,
                                                         horizon=10)

    if normals_crossing:
        bound_1_tmp = reftrack_interp[:, :2] + normvec_normalized_interp * np.expand_dims(reftrack_interp[:, 2], axis=1)
        bound_2_tmp = reftrack_interp[:, :2] - normvec_normalized_interp * np.expand_dims(reftrack_interp[:, 3], axis=1)

        plt.figure()

        plt.plot(reftrack_interp[:, 0], reftrack_interp[:, 1], 'k-')
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

        raise IOError("At least two spline normals are crossed, check input or increase smoothing factor!")

    # ------------------------------------------------------------------------------------------------------------------
    # ENFORCE MINIMUM TRACK WIDTH (INFLATE TIGHTER SECTIONS UNTIL REACHED) ---------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    manipulated_track_width = False

    if min_width is not None:
        for i in range(reftrack_interp.shape[0]):
            cur_width = reftrack_interp[i, 2] + reftrack_interp[i, 3]

            if cur_width < min_width:
                manipulated_track_width = True

                # inflate to both sides equally
                reftrack_interp[i, 2] += (min_width - cur_width) / 2
                reftrack_interp[i, 3] += (min_width - cur_width) / 2

    if manipulated_track_width:
        print("WARNING: Track region was smaller than requested minimum track width -> Applied artificial inflation in"
              " order to match the requirements!", file=sys.stderr)

    return reftrack_interp, normvec_normalized_interp, a_interp, coeffs_x_interp, coeffs_y_interp


def calc_min_bound_dists(trajectory: np.ndarray,
                         bound1: np.ndarray,
                         bound2: np.ndarray,
                         length_veh: float,
                         width_veh: float) -> np.ndarray:
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
        fl__mindist = np.sqrt(np.power(bounds[:, 0] - fl_[0], 2) + np.power(bounds[:, 1] - fl_[1], 2))
        fr__mindist = np.sqrt(np.power(bounds[:, 0] - fr_[0], 2) + np.power(bounds[:, 1] - fr_[1], 2))
        rl__mindist = np.sqrt(np.power(bounds[:, 0] - rl_[0], 2) + np.power(bounds[:, 1] - rl_[1], 2))
        rr__mindist = np.sqrt(np.power(bounds[:, 0] - rr_[0], 2) + np.power(bounds[:, 1] - rr_[1], 2))

        # save overall minimum distance of current vehicle position
        min_dists[i] = np.amin((fl__mindist, fr__mindist, rl__mindist, rr__mindist))

    return min_dists






def interp_track(reftrack: np.ndarray,
                 stepsize_approx: float = 1.0) -> np.ndarray:
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
    el_lenghts = np.sqrt(np.sum(np.power(np.diff(reftrack_cl[:, :2], axis=0), 2), axis=1))

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

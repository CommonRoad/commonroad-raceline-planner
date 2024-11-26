from typing import Union, Optional

import numpy as np
import uuid
import hashlib
from pathlib import Path


def export_traj_race(
    traj_race: np.ndarray,
    traj_race_export: Union[str, Path],
    ggv_file: Optional[Union[str, Path]] = None,
) -> None:
    """
    Created by:
    Alexander Heilmeier

    Documentation:
    This function is used to export the generated trajectory into a file. The generated files get an unique UUID and a
    hash of the ggv diagram to be able to check it later.

    Inputs:
    file_paths:     paths for input and output files {ggv_file, traj_race_export, traj_ltpl_export, lts_export}
    traj_race:      race trajectory [s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2]
    """

    # create random UUID
    rand_uuid = str(uuid.uuid4())

    # hash ggv file with SHA1
    if ggv_file is not None:
        with open(ggv_file, "br") as fh:
            ggv_content = fh.read()
    else:
        ggv_content = np.array([])
    ggv_hash = hashlib.sha1(ggv_content).hexdigest()

    # write UUID and GGV hash into file
    with open(traj_race_export, "w") as fh:
        fh.write("# " + rand_uuid + "\n")
        fh.write("# " + ggv_hash + "\n")

    # export race trajectory
    header = "s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2"
    fmt = "%.7f; %.7f; %.7f; %.7f; %.7f; %.7f; %.7f"
    with open(traj_race_export, "ab") as fh:
        np.savetxt(fh, traj_race, fmt=fmt, header=header)


def export_traj_ltpl(
    traj_ltpl_export: Union[str, Path],
    spline_lengths_opt,
    trajectory_opt,
    reftrack,
    normvec_normalized,
    alpha_opt,
    ggv_file: Optional[Union[Path, str]] = None,
) -> None:
    """
    Created by:
    Tim Stahl
    Alexander Heilmeier

    Documentation:
    This function is used to export the generated trajectory into a file for further usage in the local trajectory
    planner on the car (including map information via normal vectors and bound widths). The generated files get an
    unique UUID and a hash of the ggv diagram to be able to check it later.

    The stored trajectory has the following columns:
    [x_ref_m, y_ref_m, width_right_m, width_left_m, x_normvec_m, y_normvec_m, alpha_m, s_racetraj_m, psi_racetraj_rad,
     kappa_racetraj_radpm, vx_racetraj_mps, ax_racetraj_mps2]

    Inputs:
    file_paths:         paths for input and output files {ggv_file, traj_race_export, traj_ltpl_export, lts_export}
    spline_lengths_opt: lengths of the splines on the raceline in m
    trajectory_opt:     generated race trajectory
    reftrack:           track definition [x_m, y_m, w_tr_right_m, w_tr_left_m]
    normvec_normalized: normalized normal vectors on the reference line [x_m, y_m]
    alpha_opt:          solution vector of the opt. problem containing the lateral shift in m for every ref-point
    """

    # convert trajectory to desired format
    s_raceline_preinterp_cl = np.cumsum(spline_lengths_opt)
    s_raceline_preinterp_cl = np.insert(s_raceline_preinterp_cl, 0, 0.0)

    psi_normvec = []
    kappa_normvec = []
    vx_normvec = []
    ax_normvec = []

    for s in list(s_raceline_preinterp_cl[:-1]):
        # get closest point on trajectory_opt
        idx = (np.abs(trajectory_opt[:, 0] - s)).argmin()

        # get data at this index and append
        psi_normvec.append(trajectory_opt[idx, 3])
        kappa_normvec.append(trajectory_opt[idx, 4])
        vx_normvec.append(trajectory_opt[idx, 5])
        ax_normvec.append(trajectory_opt[idx, 6])

    traj_ltpl = np.column_stack(
        (
            reftrack,
            normvec_normalized,
            alpha_opt,
            s_raceline_preinterp_cl[:-1],
            psi_normvec,
            kappa_normvec,
            vx_normvec,
            ax_normvec,
        )
    )
    traj_ltpl_cl = np.vstack((traj_ltpl, traj_ltpl[0]))
    traj_ltpl_cl[-1, 7] = s_raceline_preinterp_cl[-1]

    # create random UUID
    rand_uuid = str(uuid.uuid4())

    # hash ggv file with SHA1
    if ggv_file is not None:
        with open(ggv_file, "br") as fh:
            ggv_content = fh.read()
    else:
        ggv_content = np.array([])
    ggv_hash = hashlib.sha1(ggv_content).hexdigest()

    # write UUID and GGV hash into file
    with open(traj_ltpl_export, "w") as fh:
        fh.write("# " + rand_uuid + "\n")
        fh.write("# " + ggv_hash + "\n")

    # export trajectory data for local planner
    header = (
        "x_ref_m; y_ref_m; width_right_m; width_left_m; x_normvec_m; y_normvec_m; "
        "alpha_m; s_racetraj_m; psi_racetraj_rad; kappa_racetraj_radpm; vx_racetraj_mps; ax_racetraj_mps2"
    )
    fmt = "%.7f; %.7f; %.7f; %.7f; %.7f; %.7f; %.7f; %.7f; %.7f; %.7f; %.7f; %.7f"
    with open(traj_ltpl_export, "ab") as fh:
        np.savetxt(fh, traj_ltpl, fmt=fmt, header=header)


def import_track(file_path: str, imp_opts: dict, width_veh: float) -> np.ndarray:
    """
    Created by:
    Alexander Heilmeier
    Modified by:
    Thomas Herrmann

    Documentation:
    This function includes the algorithm part connected to the import of the track.

    Inputs:
    file_path:      file path of track.csv containing [x_m,y_m,w_tr_right_m,w_tr_left_m]
    imp_opts:       import options showing if a new starting point should be set or if the direction should be reversed
    width_veh:      vehicle width required to check against track width

    Outputs:
    race_track:   imported track [x_m, y_m, w_tr_right_m, w_tr_left_m]
    """

    # load data from csv file
    csv_data_temp = np.loadtxt(file_path, comments="#", delimiter=",")

    # get coords and track widths out of array
    if np.shape(csv_data_temp)[1] == 3:
        refline_ = csv_data_temp[:, 0:2]
        w_tr_r = csv_data_temp[:, 2] / 2
        w_tr_l = w_tr_r

    elif np.shape(csv_data_temp)[1] == 4:
        refline_ = csv_data_temp[:, 0:2]
        w_tr_r = csv_data_temp[:, 2]
        w_tr_l = csv_data_temp[:, 3]

    elif np.shape(csv_data_temp)[1] == 5:  # omit z coordinate in this case
        refline_ = csv_data_temp[:, 0:2]
        w_tr_r = csv_data_temp[:, 3]
        w_tr_l = csv_data_temp[:, 4]

    else:
        raise IOError("Track file cannot be read!")

    refline_ = np.tile(refline_, (imp_opts["num_laps"], 1))
    w_tr_r = np.tile(w_tr_r, imp_opts["num_laps"])
    w_tr_l = np.tile(w_tr_l, imp_opts["num_laps"])

    # assemble to a single array
    reftrack_imp = np.column_stack((refline_, w_tr_r, w_tr_l))

    # check if imported centerline should be flipped, i.e. reverse direction
    if imp_opts["flip_imp_track"]:
        reftrack_imp = np.flipud(reftrack_imp)

    # check if imported centerline should be reordered for a new starting point
    if imp_opts["set_new_start"]:
        ind_start = np.argmin(
            np.power(reftrack_imp[:, 0] - imp_opts["new_start"][0], 2)
            + np.power(reftrack_imp[:, 1] - imp_opts["new_start"][1], 2)
        )
        reftrack_imp = np.roll(reftrack_imp, reftrack_imp.shape[0] - ind_start, axis=0)

    # check minimum track width for vehicle width plus a small safety margin
    w_tr_min = np.amin(reftrack_imp[:, 2] + reftrack_imp[:, 3])

    if w_tr_min < width_veh + 0.5:
        print(
            "WARNING: Minimum track width %.2fm is close to or smaller than vehicle width!"
            % np.amin(w_tr_min)
        )

    return reftrack_imp

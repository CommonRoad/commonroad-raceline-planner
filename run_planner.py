import configparser
import copy
import json
import os
import time
from dataclasses import asdict

import numpy as np
from matplotlib import pyplot as plt
from pathlib import Path

# own package
import commonroad_raceline_planner.optimization.opt_mintime_traj as opt_mintime_traj
from commonroad_raceline_planner.util.aziz_helpers.helper_functions import add_to_dict
from commonroad_raceline_planner.util.trajectory_planning_helpers.import_veh_dyn_info import import_veh_dyn_info
from commonroad_raceline_planner.optimization.opt_min_curv import opt_min_curv
from commonroad_raceline_planner.optimization.opt_shortest_path import opt_shortest_path
from commonroad_raceline_planner.util.trajectory_planning_helpers.create_raceline import create_raceline
from commonroad_raceline_planner.util.trajectory_planning_helpers.calc_vel_profile import calc_vel_profile
from commonroad_raceline_planner.util.trajectory_planning_helpers.calc_t_profile import calc_t_profile
from commonroad_raceline_planner.util.trajectory_planning_helpers.calc_head_curv_an import calc_head_curv_an
from commonroad_raceline_planner.util.trajectory_planning_helpers.calc_ax_profile import calc_ax_profile
from commonroad_raceline_planner.util.common import progressbar as tph_progressbar
from commonroad_raceline_planner.util.visualization.result_plots import result_plots
from commonroad_raceline_planner.util.trajectory_planning_helpers.calc_splines import calc_splines
from commonroad_raceline_planner.util.validation import check_traj
from commonroad_raceline_planner.configuration.race_line_config import RaceLinePlannerConfiguration

from commonroad_raceline_planner.util.io import (
    import_track,
    export_traj_ltpl,
    export_traj_race
)

from commonroad_raceline_planner.util.track_processing import prep_track



class RaceLinePlanner:
    def __init__(
            self,
            config: RaceLinePlannerConfiguration
    ):
        """
        Initialize the RaceLinePlanner with a given configuration.

        :param config: The configuration for the RaceLinePlanner.
        """
        self.pars = {}
        self.config = config
        self.file_paths = config.file_paths
        # Add module path to file_paths
        self.file_paths["module"] = os.path.dirname(os.path.abspath(__file__))
        print(os.path.dirname(os.path.abspath(__file__)))

        # Ensure paths exist
        os.makedirs(os.path.join(self.file_paths["module"], "outputs"), exist_ok=True)

    def setup_vehicle_parameters(self):
        """
        Set up the vehicle parameters by reading from the configuration file.
        """
        parser = configparser.ConfigParser()
        pars = {}

        parser.read(os.path.join(self.file_paths["module"], self.file_paths["veh_params_file"]))
        print(os.path.join(self.file_paths["module"], self.file_paths["veh_params_file"]))


        # Add attributes to dict
        add_to_dict(pars, "ggv_file", json.loads(parser.get('GENERAL_OPTIONS', 'ggv_file')))
        add_to_dict(pars, "ax_max_machines_file", json.loads(parser.get('GENERAL_OPTIONS', 'ax_max_machines_file')))
        add_to_dict(pars, "stepsize_opts", json.loads(parser.get('GENERAL_OPTIONS', 'stepsize_opts')))
        add_to_dict(pars, "reg_smooth_opts", json.loads(parser.get('GENERAL_OPTIONS', 'reg_smooth_opts')))
        add_to_dict(pars, "veh_params", json.loads(parser.get('GENERAL_OPTIONS', 'veh_params')))
        add_to_dict(pars, "vel_calc_opts", json.loads(parser.get('GENERAL_OPTIONS', 'vel_calc_opts')))

        if self.config.opt_type == 'shortest_path':
            pars["optim_opts"] = json.loads(parser.get('OPTIMIZATION_OPTIONS', 'optim_opts_shortest_path'))
        elif self.config.opt_type == 'mincurv':
            pars["optim_opts"] = json.loads(parser.get('OPTIMIZATION_OPTIONS', 'optim_opts_mincurv'))
        elif self.config.opt_type == 'mintime':
            pars["curv_calc_opts"] = json.loads(parser.get('GENERAL_OPTIONS', 'curv_calc_opts'))
            pars["optim_opts"] = json.loads(parser.get('OPTIMIZATION_OPTIONS', 'optim_opts_mintime'))
            pars["vehicle_params_mintime"] = json.loads(parser.get('OPTIMIZATION_OPTIONS', 'vehicle_params_mintime'))
            pars["tire_params_mintime"] = json.loads(parser.get('OPTIMIZATION_OPTIONS', 'tire_params_mintime'))
            pars["pwr_params_mintime"] = json.loads(parser.get('OPTIMIZATION_OPTIONS', 'pwr_params_mintime'))

            # modification of mintime options/parameters
            pars["optim_opts"]["var_friction"] = self.config.mintime_opts["var_friction"]
            pars["optim_opts"]["warm_start"] = self.config.mintime_opts["warm_start"]
            pars["vehicle_params_mintime"]["wheelbase"] = (pars["vehicle_params_mintime"]["wheelbase_front"]
                                                           + pars["vehicle_params_mintime"]["wheelbase_rear"])

        self.pars = pars

        if not (self.config.opt_type == 'mintime' and not self.config.mintime_opts['recalc_vel_profile_by_tph']):
            add_to_dict(self.file_paths, "ggv_file",
                        os.path.join(self.file_paths["module"], "inputs", "veh_dyn_info", pars["ggv_file"]))
            add_to_dict(self.file_paths, "ax_max_machines_file",
                        os.path.join(self.file_paths["module"], "inputs", "veh_dyn_info", pars["ax_max_machines_file"]))

    def import_track(self):
        """
        Import the track data from the specified file in the configuration.
        """

        # save start time
        self.t_start = time.perf_counter()

        # import track
        self.reftrack_imp = import_track(
            imp_opts=self.config.import_opts,
            file_path=self.file_paths["track_file"],
            width_veh=self.pars["veh_params"]["width"]
        )

    def import_vehicle_dynamics(self):
        """
        Import the vehicle dynamics data from the specified file in the configuration.
        """
        # import ggv and ax_max_machines (if required)
        if not (self.config.opt_type == 'mintime' and not self.config.mintime_opts['recalc_vel_profile_by_tph']):
            self.ggv, self.ax_max_machines = import_veh_dyn_info(
                ggv_import_path=self.file_paths["ggv_file"],
                ax_max_machines_import_path=self.file_paths["ax_max_machines_file"]
            )
        else:
            self.ggv = None
            self.ax_max_machines = None

    def prepare_reftrack(self):
        """
        Prepare the reference track by interpolating and normalizing the imported track data.
        """
        self.reftrack_interp, self.normvec_normalized_interp, self.a_interp, self.coeffs_x_interp, self.coeffs_y_interp = \
            prep_track(
                reftrack_imp=self.reftrack_imp,
                reg_smooth_opts=self.pars["reg_smooth_opts"],
                stepsize_opts=self.pars["stepsize_opts"],
                debug=self.config.debug['debug'],
                min_width=self.config.import_opts['min_track_width']
            )

    def optimize_trajectory(self):
        """
        Optimize the trajectory using the shortest path optimization algorithm.(TODO: Add more algorithms)
        """
        # if reoptimization of mintime solution is used afterwards we have to consider some additional deviation in the first
        # optimization
        if self.config.opt_type == 'mintime' and self.config.mintime_opts["reopt_mintime_solution"]:
            self.w_veh_tmp = self.pars["optim_opts"]["width_opt"] + (
                    self.pars["optim_opts"]["w_tr_reopt"] - self.pars["optim_opts"]["w_veh_reopt"])
            self.w_veh_tmp += self.pars["optim_opts"]["w_add_spl_regr"]
            self.pars_tmp = copy.deepcopy(self.pars)
            self.pars_tmp["optim_opts"]["width_opt"] = self.w_veh_tmp
        else:
            self.pars_tmp = self.pars

        # Call optimization
        if self.config.opt_type == 'shortest_path':
            self.alpha_opt = opt_shortest_path(
                reftrack=self.reftrack_interp,
                normvectors=self.normvec_normalized_interp,
                w_veh=self.pars["optim_opts"]["width_opt"],
                print_debug=self.config.debug['debug']
            )
        elif self.config.opt_type == 'mincurv':
            self.alpha_opt = opt_min_curv(
                reftrack=self.reftrack_interp,
                normvectors=self.normvec_normalized_interp,
                A=self.a_interp,
                kappa_bound=self.pars["veh_params"]["curvlim"],
                w_veh=self.pars["optim_opts"]["width_opt"],
                print_debug=self.config.debug['debug'],
                plot_debug=self.config.debug['plot_opts']["mincurv_curv_lin"]
            )[0]
        elif self.config.opt_type == 'mintime':
            # reftrack_interp, a_interp and normvec_normalized_interp are returned for the case that non-regular sampling was
            # applied
            self.alpha_opt, self.v_opt, self.reftrack_interp, self.a_interp_tmp, self.normvec_normalized_interp = opt_mintime_traj.src.opt_mintime. \
                opt_mintime(reftrack=self.reftrack_interp,
                            coeffs_x=self.coeffs_x_interp,
                            coeffs_y=self.coeffs_y_interp,
                            normvectors=self.normvec_normalized_interp,
                            pars=self.pars_tmp,
                            tpamap_path=self.config.file_paths["tpamap"],
                            tpadata_path=self.config.file_paths["tpadata"],
                            export_path=self.config.file_paths["mintime_export"],
                            print_debug=self.config.debug['debug'],
                            plot_debug=self.config.debug['plot_opts']["mintime_plots"]
                            )

            # replace a_interp if necessary
            if self.a_interp_tmp is not None:
                self.a_interp = self.a_interp_tmp
            else:
                raise ValueError('Unknown optimization type!')

        if self.config.opt_type == 'mintime' and self.config.mintime_opts["reopt_mintime_solution"]:

            # get raceline solution of the time-optimal trajectory
            self.raceline_mintime = self.reftrack_interp[:, :2] + np.expand_dims(self.alpha_opt, 1) * self.normvec_normalized_interp

            # calculate new track boundaries around raceline solution depending on alpha_opt values
            self.w_tr_right_mintime = self.reftrack_interp[:, 2] - self.alpha_opt
            self.w_tr_left_mintime = self.reftrack_interp[:, 3] + self.alpha_opt

            # create new reference track around the raceline
            self.racetrack_mintime = np.column_stack((self.raceline_mintime, self.w_tr_right_mintime, self.w_tr_left_mintime))

            # use spline approximation a second time
            self.reftrack_interp, self.normvec_normalized_interp, self.a_interp = \
                prep_track(reftrack_imp=self.racetrack_mintime,
                                                            reg_smooth_opts=self.pars["reg_smooth_opts"],
                                                            stepsize_opts=self.pars["stepsize_opts"],
                                                            debug=False,
                                                            min_width=self.config.import_opts["min_track_width"])[:3]

            # set artificial track widths for reoptimization
            self.w_tr_tmp = 0.5 * self.pars["optim_opts"]["w_tr_reopt"] * np.ones(self.reftrack_interp.shape[0])
            self. racetrack_mintime_reopt = np.column_stack((self.reftrack_interp[:, :2], self.w_tr_tmp, self.w_tr_tmp))

            # call mincurv reoptimization
            self.alpha_opt = opt_min_curv(reftrack=self.racetrack_mintime_reopt,
                                                      normvectors=self.normvec_normalized_interp,
                                                      A=self.a_interp,
                                                      kappa_bound=self.pars["veh_params"]["curvlim"],
                                                      w_veh=self.pars["optim_opts"]["w_veh_reopt"],
                                                      print_debug=self.config.debug['debug'],
                                                      plot_debug=self.config.debug['plot_opts']["mincurv_curv_lin"])[0]

            # calculate minimum distance from raceline to bounds and print it
            if self.config.debug:
                self.raceline_reopt = self.reftrack_interp[:, :2] + np.expand_dims(self.alpha_opt, 1) * self.normvec_normalized_interp
                self.bound_r_reopt = (self.reftrack_interp[:, :2]
                                 + np.expand_dims(self.reftrack_interp[:, 2], axis=1) * self.normvec_normalized_interp)
                self.bound_l_reopt = (self.reftrack_interp[:, :2]
                                 - np.expand_dims(self.reftrack_interp[:, 3], axis=1) * self.normvec_normalized_interp)

                self.d_r_reopt = np.hypot(self.raceline_reopt[:, 0] - self.bound_r_reopt[:, 0],
                                     self.raceline_reopt[:, 1] - self.bound_r_reopt[:, 1])
                self.d_l_reopt = np.hypot(self.raceline_reopt[:, 0] - self.bound_l_reopt[:, 0],
                                     self.raceline_reopt[:, 1] - self.bound_l_reopt[:, 1])

                print("INFO: Mintime reoptimization: minimum distance to right/left bound: %.2fm / %.2fm"
                      % (np.amin(self.d_r_reopt) - self.pars["veh_params"]["width"] / 2,
                         np.amin(self.d_l_reopt) - self.pars["veh_params"]["width"] / 2))

    def interpolate_raceline(self):
        """
        Interpolate the raceline using the optimized trajectory.
        """
        self.raceline_interp, self.a_opt, self.coeffs_x_opt, self.coeffs_y_opt, \
            self.spline_inds_opt_interp, self.t_vals_opt_interp, self.s_points_opt_interp, \
            self.spline_lengths_opt, self.el_lengths_opt_interp = create_raceline(
            refline=self.reftrack_interp[:, :2],
            normvectors=self.normvec_normalized_interp,
            alpha=self.alpha_opt,
            stepsize_interp=self.pars["stepsize_opts"]["stepsize_interp_after_opt"]
        )

    def calculate_heading_and_curvature(self):
        """
        Calculate the heading and curvature of the raceline.
        """
        # calculate heading and curvature (analytically)
        self.psi_vel_opt, self.kappa_opt = calc_head_curv_an(
            coeffs_x=self.coeffs_x_opt,
            coeffs_y=self.coeffs_y_opt,
            ind_spls=self.spline_inds_opt_interp,
            t_spls=self.t_vals_opt_interp
        )

    def calculate_velocity_profile(self):
        """
        Calculate the velocity profile of the raceline.
        """
        self.vx_profile_opt = calc_vel_profile(
            ggv=self.ggv,
            ax_max_machines=self.ax_max_machines,
            v_max=self.pars["veh_params"]["v_max"],
            kappa=self.kappa_opt,
            el_lengths=self.el_lengths_opt_interp,
            closed=True,
            filt_window=self.pars["vel_calc_opts"]["vel_profile_conv_filt_window"],
            dyn_model_exp=self.pars["vel_calc_opts"]["dyn_model_exp"],
            drag_coeff=self.pars["veh_params"]["dragcoeff"],
            m_veh=self.pars["veh_params"]["mass"]
        )

        # calculate longitudinal acceleration profile
        self.vx_profile_opt_cl = np.append(self.vx_profile_opt, self.vx_profile_opt[0])
        self.ax_profile_opt = calc_ax_profile(
            vx_profile=self.vx_profile_opt_cl,
            el_lengths=self.el_lengths_opt_interp,
            eq_length_output=False
        )

        # calculate laptime
        self.t_profile_cl = calc_t_profile(
            vx_profile=self.vx_profile_opt,
            ax_profile=self.ax_profile_opt,
            el_lengths=self.el_lengths_opt_interp
        )
        print("INFO: Estimated laptime: %.2fs" % self.t_profile_cl[-1])

        if self.config.debug['plot_opts']["racetraj_vel"]:
            s_points = np.cumsum(self.el_lengths_opt_interp[:-1])
            s_points = np.insert(s_points, 0, 0.0)

            plt.plot(s_points, self.vx_profile_opt)
            plt.plot(s_points, self.ax_profile_opt)
            plt.plot(s_points, self.t_profile_cl[:-1])

            plt.grid()
            plt.xlabel("distance in m")
            plt.legend(["vx in m/s", "ax in m/s2", "t in s"])

            plt.show()

    def calculate_lap_time_matrix(self):
        """
        Calculate the lap time matrix for the raceline.
        """
        ggv_scales = np.linspace(
            self.config.lap_time_mat_opts.gg_scale_range[0],
            self.config.lap_time_mat_opts.gg_scale_range[1],
            int((self.config.lap_time_mat_opts.gg_scale_range[1] - self.config.lap_time_mat_opts.gg_scale_range[0])
                / self.config.lap_time_mat_opts.gg_scale_stepsize) + 1
        )
        top_speeds = np.linspace(
            self.config.lap_time_mat_opts.top_speed_range[0] / 3.6,
            self.config.lap_time_mat_opts.top_speed_range[1] / 3.6,
            int((self.config.lap_time_mat_opts.top_speed_range[1] - self.config.lap_time_mat_opts.top_speed_range[0])
                / self.config.lap_time_mat_opts.top_speed_stepsize) + 1
        )

        # setup results matrix
        lap_time_matrix = np.zeros((top_speeds.shape[0] + 1, ggv_scales.shape[0] + 1))

        # write parameters in first column and row
        lap_time_matrix[1:, 0] = top_speeds * 3.6
        lap_time_matrix[0, 1:] = ggv_scales

        for i, top_speed in enumerate(top_speeds):
            for j, ggv_scale in enumerate(ggv_scales):
                tph_progressbar(
                    i * ggv_scales.shape[0] + j,
                    top_speeds.shape[0] * ggv_scales.shape[0],
                    prefix="Simulating laptimes "
                )

                ggv_mod = np.copy(self.ggv)
                ggv_mod[:, 1:] *= ggv_scale

                vx_profile_opt = calc_vel_profile(
                    ggv=ggv_mod,
                    ax_max_machines=self.ax_max_machines,
                    v_max=top_speed,
                    kappa=self.kappa_opt,
                    el_lengths=self.el_lengths_opt_interp,
                    dyn_model_exp=self.pars["vel_calc_opts"]["dyn_model_exp"],
                    filt_window=self.pars["vel_calc_opts"]["vel_profile_conv_filt_window"],
                    closed=True,
                    drag_coeff=self.pars["veh_params"]["dragcoeff"],
                    m_veh=self.pars["veh_params"]["mass"]
                )

                # calculate longitudinal acceleration profile
                vx_profile_opt_cl = np.append(vx_profile_opt, vx_profile_opt[0])
                ax_profile_opt = calc_ax_profile(
                    vx_profile=vx_profile_opt_cl,
                    el_lengths=self.el_lengths_opt_interp,
                    eq_length_output=False
                )

                # calculate laptime
                t_profile_cl = calc_t_profile(
                    vx_profile=vx_profile_opt,
                    ax_profile=ax_profile_opt,
                    el_lengths=self.el_lengths_opt_interp
                )

                # store entry in lap time matrix
                lap_time_matrix[i + 1, j + 1] = t_profile_cl[-1]

        # store lap time matrix to file
        np.savetxt(self.file_paths["lap_time_mat_export"], lap_time_matrix, delimiter=",", fmt="%.3f")

    def postprocess_data(self):
        """
        Postprocess the data after optimization.
        """
        # arrange data into one trajectory
        self.trajectory_opt = np.column_stack((self.s_points_opt_interp,
                                               self.raceline_interp,
                                               self.psi_vel_opt,
                                               self.kappa_opt,
                                               self.vx_profile_opt,
                                               self.ax_profile_opt))
        spline_data_opt = np.column_stack((self.spline_lengths_opt, self.coeffs_x_opt, self.coeffs_y_opt))

        # create a closed race trajectory array
        self.traj_race_cl = np.vstack((self.trajectory_opt, self.trajectory_opt[0, :]))
        self.traj_race_cl[-1, 0] = np.sum(spline_data_opt[:, 0])  # set correct length

        # print end time
        print("INFO: Runtime from import to final trajectory was %.2fs" % (time.perf_counter() - self.t_start))

    def check_trajectory(self):
        """
        Check the trajectory for errors.
        """
        self.bound1, self.bound2 = check_traj(reftrack=self.reftrack_interp,
                       reftrack_normvec_normalized=self.normvec_normalized_interp,
                       length_veh=self.pars["veh_params"]["length"],
                       width_veh=self.pars["veh_params"]["width"],
                       debug=self.config.debug['debug'],
                       trajectory=self.trajectory_opt,
                       ggv=self.ggv,
                       ax_max_machines=self.ax_max_machines,
                       v_max=self.pars["veh_params"]["v_max"],
                       curvlim=self.pars["veh_params"]["curvlim"],
                       mass_veh=self.pars["veh_params"]["mass"],
                       dragcoeff=self.pars["veh_params"]["dragcoeff"])

    def export_trajectory(self):
        """
        Export the trajectory to the specified file in the configuration.
        """
        # checks if files_paths is a dictionary or a dataclass and converts it to a dictionary
        if isinstance(self.file_paths, dict):
            files_paths_asdict = self.file_paths
        else:
            files_paths_asdict = asdict(self.file_paths)

        # export race trajectory  to CSV
        if "traj_race_export" in files_paths_asdict.keys():
            export_traj_race(file_paths=files_paths_asdict,
                                                                    traj_race=self.traj_race_cl)

        # if requested, export trajectory including map information (via normal vectors) to CSV
        if "traj_ltpl_export" in files_paths_asdict.keys():
            export_traj_ltpl(files_paths_asdict,
                                                                    spline_lengths_opt=self.spline_lengths_opt,
                                                                    trajectory_opt=self.trajectory_opt,
                                                                    reftrack=self.reftrack_interp,
                                                                    normvec_normalized=self.normvec_normalized_interp,
                                                                    alpha_opt=self.alpha_opt)
        print("INFO: Finished export of trajectory:", time.strftime("%H:%M:%S"))

    def plot_results(self):
        """
        Plot the results of the trajectory optimization.
        """
        # get bound of imported map (for reference in final plot)
        bound1_imp = None
        bound2_imp = None

        if self.config.debug['plot_opts']["imported_bounds"]:
            # try to extract four times as many points as in the interpolated version (in order to hold more details)
            n_skip = max(int(self.reftrack_imp.shape[0] / (self.bound1.shape[0] * 4)), 1)

            _, _, _, normvec_imp = calc_splines(path=np.vstack((self.reftrack_imp[::n_skip, 0:2],
                                                                                 self.reftrack_imp[0, 0:2])))

            bound1_imp = self.reftrack_imp[::n_skip, :2] + normvec_imp * np.expand_dims(self.reftrack_imp[::n_skip, 2],
                                                                                        1)
            bound2_imp = self.reftrack_imp[::n_skip, :2] - normvec_imp * np.expand_dims(self.reftrack_imp[::n_skip, 3],
                                                                                        1)

        # plot results
        result_plots(plot_opts=self.config.debug['plot_opts'],
                                                        width_veh_opt=self.pars["optim_opts"]["width_opt"],
                                                        width_veh_real=self.pars["veh_params"]["width"],
                                                        refline=self.reftrack_interp[:, :2],
                                                        bound1_imp=bound1_imp,
                                                        bound2_imp=bound2_imp,
                                                        bound1_interp=self.bound1,
                                                        bound2_interp=self.bound2,
                                                        trajectory=self.trajectory_opt)

        # print("First element of trajectory_opt:", self.trajectory_opt[0])
        # print("Second element of trajectory_opt:", self.trajectory_opt[1])

    def export_reference_path(self):
        """
        Export the reference path to a structure that can be used by ReactivePlanner.
        """
        reference_path = {
            'x': self.raceline_interp[:, 0].tolist(),
            'y': self.raceline_interp[:, 1].tolist()
        }
        with open(self.file_paths["reference_path_export"], 'w') as f:
            json.dump(reference_path, f)
        print("INFO: Finished export of reference path:", time.strftime("%H:%M:%S"))

    def export_velocity_profile(self):
        """
        Export the velocity profile to a JSON file.
        """
        velocity_profile = {
            'vx': self.vx_profile_opt.tolist(),
            't': self.t_profile_cl.tolist()
        }
        with open(self.file_paths["velocity_profile_export"], 'w') as f:
            json.dump(velocity_profile, f)
        print("INFO: Finished export of velocity profile:", time.strftime("%H:%M:%S"))

    def run(self):
        """
        Run the RaceLinePlanner.
        """
        self.setup_vehicle_parameters()
        self.import_track()
        self.import_vehicle_dynamics()
        self.prepare_reftrack()
        self.optimize_trajectory()
        self.interpolate_raceline()
        self.calculate_heading_and_curvature()
        self.calculate_velocity_profile()

        if self.config.lap_time_mat_opts['use_lap_time_mat']:
            self.calculate_lap_time_matrix()
        self.postprocess_data()
        self.check_trajectory()
        self.export_trajectory()
        self.export_reference_path()
        self.export_velocity_profile()
        self.plot_results()


if __name__ == "__main__":
    config_path: Path = Path(__file__).parents[0] / "configurations/race_line_planner_config.yaml"

    config = RaceLinePlannerConfiguration.load(config_path)
    planner = RaceLinePlanner(config)
    planner.run()

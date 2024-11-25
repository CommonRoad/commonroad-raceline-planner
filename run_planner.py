import json
import os
import time
from typing import Union

import numpy as np
from matplotlib import pyplot as plt
from pathlib import Path

from commonroad_raceline_planner.configuration.ftm_config.computation_config import ComputationConfig, ComputationConfigFactory
# own package
from commonroad_raceline_planner.dataloader.racetrack_factory import RaceTrackFactory
from commonroad_raceline_planner.raceline import RaceLine, RaceLineFactory
from commonroad_raceline_planner.ractetrack import DtoFTM, DtoFTMFactory
from commonroad_raceline_planner.util.trajectory_planning_helpers.import_veh_dyn_info import import_ggv_diagram, \
    import_engine_constraints
from commonroad_raceline_planner.optimization.opt_min_curv import opt_min_curv
from commonroad_raceline_planner.optimization.opt_shortest_path import opt_shortest_path
from commonroad_raceline_planner.util.trajectory_planning_helpers.create_raceline import create_raceline
from commonroad_raceline_planner.planner.ftm_planner.velenis_vel_profile import calc_vel_profile
from commonroad_raceline_planner.util.trajectory_planning_helpers.calc_t_profile import calc_t_profile
from commonroad_raceline_planner.util.trajectory_planning_helpers.calc_head_curv_an import calc_head_curv_an
from commonroad_raceline_planner.util.trajectory_planning_helpers.calc_ax_profile import calc_ax_profile
from commonroad_raceline_planner.util.common import progressbar as tph_progressbar
from commonroad_raceline_planner.util.visualization.result_plots import result_plots, plot_cr_results
from commonroad_raceline_planner.util.trajectory_planning_helpers.calc_splines import calc_splines
from commonroad_raceline_planner.util.validation import check_traj
from commonroad_raceline_planner.configuration.ftm_config.execution_config import ExecutionConfig, ExecutionConfigFactory
from commonroad_raceline_planner.configuration.ftm_config.optimization_config import OptimizationType

from commonroad.common.file_reader import CommonRoadFileReader

from commonroad_raceline_planner.util.io import (
    export_traj_ltpl,
    export_traj_race,
)

from commonroad_raceline_planner.util.track_processing import preprocess_track



class RaceLinePlanner:
    def __init__(
            self,
            execution_config: ExecutionConfig
    ):
        """
        Initialize the RaceLinePlanner with a given configuration.

        :param execution_config: The configuration for the RaceLinePlanner.
        """
        self.pars = {}
        self.pars_tmp = None
        self.alpha_opt = None
        self._computation_config: ComputationConfig = None
        self._execution_config = execution_config
        self._module_path: Union[str, Path] = os.path.dirname(os.path.abspath(__file__))

        self.dto_race_track: DtoFTM = None

        # Ensure paths exist
        os.makedirs(os.path.join(self._module_path, "outputs"), exist_ok=True)

    def import_stuff(self):
        """
        Set up the vehicle parameters by reading from the configuration file.
        """
        # vehicle params
        print(f"loading vehicle params")
        path_to_racecar_ini = os.path.join(self._module_path, self._execution_config.filepath_config.veh_params_file)
        self._computation_config = ComputationConfigFactory().generate_from_racecar_ini(
            path_to_racecar_ini=path_to_racecar_ini
        )

        # racetrack
        print(f"import race track")
        self.race_track_csv = RaceTrackFactory().generate_racetrack_from_csv(
            file_path=self._execution_config.filepath_config.track_file,
            vehicle_width=self._computation_config.general_config.vehicle_config.width
        )

        # cr-io
        cr_path = "/home/tmasc/projects/cr-raceline/commonroad-raceline-planner/inputs/tracks/XML_maps/DEU_Hhr-1_1.xml"
        self.scenario, planning_problem_set = CommonRoadFileReader(cr_path).open()
        self.planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]
        self.race_track = RaceTrackFactory().generate_racetrack_from_cr_scenario(
            lanelet_network=self.scenario.lanelet_network,
            vehicle_width=self._computation_config.general_config.vehicle_config.width
        )
        self.dto_race_track = DtoFTMFactory().generate_from_racetrack(race_track=self.race_track)

        # Vehicle dynamics
        self.t_start = time.perf_counter()
        print(f"ggv loaded")
        if not (
                self._execution_config.optimization_type == OptimizationType.MINIMUM_LAPTIME
                and not self._execution_config.mintime_config.recalc_vel_profile_by_tph
        ):
            self.ggv = import_ggv_diagram(
                ggv_import_path=self._execution_config.filepath_config.ggv_file,
            )
            self.ax_max_machines = import_engine_constraints(
                ax_max_machines_import_path=self._execution_config.filepath_config.ax_max_machines_file
            )
        else:
            print(f"no ggv loaded")
            self.ggv = None
            self.ax_max_machines = None


    def prepare_reftrack(self):
        """
        Prepare the track by interpolating and normalizing the imported track data.
        """
        self.spline_track, self.normvec_normalized_interp, self.a_interp, self.coeffs_x_interp, self.coeffs_y_interp = \
            preprocess_track(
                race_track=self.dto_race_track,
                k_reg=self._computation_config.general_config.smoothing_config.k_reg,
                s_reg=self._computation_config.general_config.smoothing_config.s_reg,
                debug=self._execution_config.debug_config.debug,
                min_width=self._execution_config.import_config.min_track_width,
                stepsize_prep=self._computation_config.general_config.stepsize_config.stepsize_preperation,
                stepsize_reg=self._computation_config.general_config.stepsize_config.stepsize_regression
            )

    def optimize_trajectory(self) -> None:
        """
        Optimize the trajectory using the shortest path optimization algorithm.(TODO: Add more algorithms)
        """
        print(f"start optimization")

        self.pars_tmp = self.pars

        # Shortest path optimization
        if self._execution_config.optimization_type == OptimizationType.SHORTEST_PATH:
            print(f'shortest path')
            self.alpha_opt = opt_shortest_path(
                reftrack=self.spline_track,
                normvectors=self.normvec_normalized_interp,
                vehicle_width=self._computation_config.optimization_config.opt_shortest_path_config.vehicle_width_opt,
                print_debug=self._execution_config.debug_config.debug
            )

        # Minimum curvature optimization
        elif self._execution_config.optimization_type == OptimizationType.MINIMUM_CURVATURE:
            print(f'minimum curvature')
            self.alpha_opt, maximum_curvature_error = opt_min_curv(
                reftrack=self.spline_track,
                normvectors=self.normvec_normalized_interp,
                A=self.a_interp,
                kappa_bound=self._computation_config.general_config.vehicle_config.curvature_limit,
                w_veh=self._computation_config.optimization_config.opt_min_curvature_config.vehicle_width_opt,
                print_debug=self._execution_config.debug_config.debug,
                plot_debug=self._execution_config.debug_config.mincurv_curv_lin
            )
        else:
            raise NotImplementedError(f'optimization type {self._execution_config.optimization_type} not known.')

        print(f"end optimization")


    def interpolate_raceline(self):
        """
        Interpolate the raceline using the optimized trajectory.
        """
        self.spline_track.open_racetrack()
        self.raceline_interp, self.a_opt, self.coeffs_x_opt, self.coeffs_y_opt, \
            self.spline_inds_opt_interp, self.t_vals_opt_interp, self.s_points_opt_interp, \
            self.spline_lengths_opt, self.el_lengths_opt_interp = create_raceline(
            refline=self.spline_track.to_2d_np_array(),
            normvectors=self.normvec_normalized_interp,
            alpha=self.alpha_opt,
            stepsize_interp=self._computation_config.general_config.stepsize_config.stepsize_interp_after_opt
        )
        self.spline_track.close_racetrack()

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
            v_max=self._computation_config.general_config.vehicle_config.v_max,
            kappa=self.kappa_opt,
            el_lengths=self.el_lengths_opt_interp,
            closed=True,
            filt_window=self._computation_config.general_config.velocity_calc_config.velocity_profile_filter,
            dyn_model_exp=self._computation_config.general_config.velocity_calc_config.dyn_model_exp,
            drag_coeff=self._computation_config.general_config.vehicle_config.drag_coefficient,
            m_veh=self._computation_config.general_config.vehicle_config.mass
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

        if self._execution_config.debug_config.racetraj_vel:
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
            self._execution_config.lap_time_matrix_config.gg_scale_range[0],
            self._execution_config.lap_time_matrix_config.gg_scale_range[1],
            int((self._execution_config.lap_time_matrix_config.gg_scale_range[1] - self._execution_config.lap_time_matrix_config.gg_scale_range[0])
                / self._execution_config.lap_time_matrix_config.gg_scale_stepsize) + 1
        )
        top_speeds = np.linspace(
            self._execution_config.lap_time_matrix_config.top_speed_range[0] / 3.6,
            self._execution_config.lap_time_matrix_config.top_speed_range[1] / 3.6,
            int((self._execution_config.lap_time_matrix_config.top_speed_range[1] - self._execution_config.lap_time_matrix_config.top_speed_range[0])
                / self._execution_config.lap_time_matrix_config.top_speed_stepsize) + 1
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
                    dyn_model_exp=self._computation_config.general_config.velocity_calc_config.dyn_model_exp,
                    filt_window=self._computation_config.general_config.velocity_calc_config.velocity_profile_filter,
                    closed=True,
                    drag_coeff=self._computation_config.general_config.vehicle_config.drag_coefficient,
                    m_veh=self._computation_config.general_config.vehicle_config.mass
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
        np.savetxt(self._execution_config.filepath_config.lap_time_mat_export, lap_time_matrix, delimiter=",", fmt="%.3f")

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

        # TODO: ???
        # create a closed race trajectory array
        self.traj_race_cl = np.vstack((self.trajectory_opt, self.trajectory_opt[0, :]))
        self.traj_race_cl[-1, 0] = np.sum(spline_data_opt[:, 0])  # set correct length

        # print end time
        print("INFO: Runtime from import to final trajectory was %.2fs" % (time.perf_counter() - self.t_start))

    def check_trajectory(self):
        """
        Check the trajectory for errors.
        """
        self.spline_track.open_racetrack()
        self.bound1, self.bound2 = check_traj(
              reftrack=self.spline_track,
              reftrack_normvec_normalized=self.normvec_normalized_interp,
              length_veh=self._computation_config.general_config.vehicle_config.length,
              width_veh=self._computation_config.general_config.vehicle_config.width,
              debug=self._execution_config.debug_config.debug,
              trajectory=self.trajectory_opt,
              ggv=self.ggv,
              ax_max_machines=self.ax_max_machines,
              v_max=self._computation_config.general_config.vehicle_config.v_max,
              curvlim=self._computation_config.general_config.vehicle_config.curvature_limit,
              mass_veh=self._computation_config.general_config.vehicle_config.mass,
              dragcoeff=self._computation_config.general_config.vehicle_config.drag_coefficient
        )
        self.spline_track.close_racetrack()

    def generate_trajectories(self):
        """
        Export the trajectory to the specified file in the configuration.
        """

        # export race trajectory  to CSV
        if self._execution_config.filepath_config.traj_race_export is not None:
            export_traj_race(
                traj_race_export=self._execution_config.filepath_config.traj_race_export,
                ggv_file=self._execution_config.filepath_config.ggv_file,
                traj_race=self.traj_race_cl
            )

        # if requested, export trajectory including map information (via normal vectors) to CSV
        if self._execution_config.filepath_config.traj_ltpl_export is not None:
            export_traj_ltpl(
                traj_ltpl_export=self._execution_config.filepath_config.traj_ltpl_export,
                ggv_file=self._execution_config.filepath_config.ggv_file,
                spline_lengths_opt=self.spline_lengths_opt,
                trajectory_opt=self.trajectory_opt,
                reftrack=self.spline_track,
                normvec_normalized=self.normvec_normalized_interp,
                alpha_opt=self.alpha_opt
            )
        print("INFO: Finished export of trajectory:", time.strftime("%H:%M:%S"))

        self.race_line: RaceLine = RaceLineFactory().generate_raceline(
            length_per_point=self.s_points_opt_interp,
            points=self.raceline_interp,
            velocity_long_per_point=self.vx_profile_opt,
            acceleration_long_per_point=self.ax_profile_opt,
            curvature_per_point=self.kappa_opt,
            heading_per_point=self.kappa_opt,
            closed=True
        )

    def plot_results(self):
        """
        Plot the results of the trajectory optimization.
        """
        # get bound of imported map (for reference in final plot)
        bound1_imp = None
        bound2_imp = None

        if self._execution_config.debug_config.imported_bounds:
            # try to extract four times as many points as in the interpolated version (in order to hold more details)
            n_skip = max(int(self.race_track.num_points / (self.bound1.shape[0] * 4)), 1)

            _, _, _, normvec_imp = calc_splines(
                path=np.vstack(
                    (self.race_track.to_2d_np_array()[::n_skip], self.race_track.to_2d_np_array()[0])
                )
            )

            bound1_imp = self.race_track.w_tr_right_m[::n_skip] + normvec_imp * np.expand_dims(
                self.race_track.w_tr_right_m[::n_skip],
                axis=1
            )
            bound2_imp = self.race_track.w_tr_right_m[::n_skip] - normvec_imp * np.expand_dims(
                self.race_track.w_tr_left_m[::n_skip],
                axis=1
            )

        # plot results
        result_plots(
            plot_imported_bounds=self._execution_config.debug_config.imported_bounds,
            plot_raceline=self._execution_config.debug_config.raceline,
            plot_racetraj_vel_3d=self._execution_config.debug_config.racetraj_vel_3d,
            plot_racline_curvature=self._execution_config.debug_config.raceline_curv,
            plot_spline_normals=self._execution_config.debug_config.spline_normals,
            racetraj_vel_3d_stepsize=self._execution_config.debug_config.racetraj_vel_3d_stepsize,
            width_veh_opt=self._computation_config.optimization_config.opt_min_curvature_config.vehicle_width_opt,
            width_veh_real=self._computation_config.general_config.vehicle_config.width,
            refline=self.spline_track.to_2d_np_array(),
            bound1_imp=bound1_imp,
            bound2_imp=bound2_imp,
            bound1_interp=self.bound1,
            bound2_interp=self.bound2,
            trajectory=self.trajectory_opt
        )

        if self.race_track.lanelet_network is not None:
            plot_cr_results(
                race_line=self.race_line,
                lanelet_network=self.scenario.lanelet_network,
                planning_problem=self.planning_problem
            )


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
        with open(self._execution_config.filepath_config.reference_path_export, 'w') as f:
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
        with open(self._execution_config.filepath_config.velocity_profile_export, 'w') as f:
            json.dump(velocity_profile, f)
        print("INFO: Finished export of velocity profile:", time.strftime("%H:%M:%S"))



    def run(self):
        """
        Run the RaceLinePlanner.
        """
        print("1")
        self.import_stuff()
        print("4")
        self.prepare_reftrack()
        print("5")
        self.optimize_trajectory()
        print("6")
        self.interpolate_raceline()
        print("7")
        self.calculate_heading_and_curvature()
        print("8")
        self.calculate_velocity_profile()
        print("9")

        if self._execution_config.lap_time_matrix_config.use_lap_time_mat:
            self.calculate_lap_time_matrix()
            print("10")
        self.postprocess_data()
        print("11")
        self.check_trajectory()
        print("12")
        self.generate_trajectories()
        print("13")
        self.export_reference_path()
        print("14")
        self.export_velocity_profile()
        print("15")
        self.plot_results()


if __name__ == "__main__":
    config_path: Path = Path(__file__).parents[0] / "configurations/race_line_planner_config.yaml"

    config: ExecutionConfig = ExecutionConfigFactory().generate_from_yml(config_path)
    planner = RaceLinePlanner(config)
    planner.run()

import logging
from logging import Logger
from typing import Any

import numpy as np

from commonroad_raceline_planner.configuration.ftm_config.ftm_config import FTMConfig
from commonroad_raceline_planner.optimization.opt_min_curv import opt_min_curv
from commonroad_raceline_planner.planner.base_planner import BaseRacelinePlanner
from commonroad_raceline_planner.planner.ftm_planner.velenis_vel_profile import calc_vel_profile
from commonroad_raceline_planner.raceline import RaceLine
from commonroad_raceline_planner.racetrack_layers.lin_interpol_layer import LinearInterpolationLayer
from commonroad_raceline_planner.racetrack_layers.spline_approx_layer import SplineApproxLayer
from commonroad_raceline_planner.racetrack_layers.width_inflation_layer import WidthInflationLayer
from commonroad_raceline_planner.ractetrack import RaceTrack, DtoFTM, DtoFTMFactory
from commonroad_raceline_planner.util.track_processing import compute_normals_and_check_crosing
from commonroad_raceline_planner.util.trajectory_planning_helpers.calc_ax_profile import calc_ax_profile
from commonroad_raceline_planner.util.trajectory_planning_helpers.calc_head_curv_an import calc_head_curv_an
from commonroad_raceline_planner.util.trajectory_planning_helpers.calc_t_profile import calc_t_profile
from commonroad_raceline_planner.util.trajectory_planning_helpers.create_raceline import create_raceline
from commonroad_raceline_planner.util.trajectory_planning_helpers.import_veh_dyn_info import import_ggv_diagram, \
    import_engine_constraints



class MinimumCurvaturePlanner(BaseRacelinePlanner):

    def __init__(
            self,
            race_track: RaceTrack,
            config: FTMConfig,
            logger_level: int = logging.INFO
    ) -> None:
        super().__init__(
            race_track=race_track,
            config=config
        )

        # logger
        self._logger: Logger = Logger("FTM_MinCurv")
        self._logger.setLevel(logger_level)

        # import vehicle dynamics info
        self._ggv: np.ndarray = None
        self._engine_constraints: np.ndarray = None
        self.update_config(config=config)

        # Data transfer object
        self._dto: DtoFTM = DtoFTMFactory().generate_from_racetrack(race_track)

        # Preprocessing
        self._inflated_track: DtoFTM = None
        self._normvec_normalized_interp: np.ndarray = None
        self._a_interp: np.ndarray = None
        self._coeffs_x_interp: np.ndarray = None
        self._coeffs_y_interp: np.ndarray = None

        # optimization
        self._alpha_opt: np.ndarray = None
        self._maximum_curvature_error: float = None

        # positional raceline calculation
        self._raceline_interp: np.ndarray = None
        self._a_opt: np.ndarray = None
        self._coeffs_x_opt: np.ndarray = None
        self._coeffs_y_opt: np.ndarray = None
        self._spline_inds_opt_interp: np.ndarray = None
        self._t_vals_opt_interp: np.ndarray = None
        self._s_points_opt_interp: np.ndarray = None
        self._spline_lengths_opt: np.ndarray = None
        self._el_lengths_opt_interp: np.ndarray = None
        self._psi_vel_opt: float = None
        self._kappa_opt: float = None

        # velocity information
        self._vx_profile_opt: np.ndarray = None
        self._vx_profile_opt_cl: np.ndarray = None
        self._ax_profile_opt: np.ndarray = None
        self._t_profile_cl: np.ndarray = None


    @property
    def logger(self) -> Logger:
        """
        :return: logger
        """
        return self._logger

    @property
    def ggv(self) -> np.ndarray:
        """
        :return: ggv diagram as np.ndarray
        """
        return self._ggv

    @property
    def engine_constraints(self) -> np.ndarray:
        """
        :return: engine constraints as np.ndarray
        """
        return self._engine_constraints

    @property
    def maximum_curvature_error(self) -> float:
        """
        :return: maximum curvature error between linearization and original
        """
        return self._maximum_curvature_error

    def update_config(
            self,
            config: FTMConfig
    ) -> None:
        """
        Updates config
        :param config: FTM Config
        """
        self._config: FTMConfig = config
        self._ggv: np.ndarray = import_ggv_diagram(ggv_import_path=config.execution_config.filepath_config.ggv_file)
        self._engine_constraints: np.ndarray = import_engine_constraints(
            ax_max_machines_import_path=config.execution_config.filepath_config.ax_max_machines_file
        )


    def plan(self) -> RaceLine:
        """
        Runs raceline planner
        :return: cr raceline object
        """
        # preprocessing
        self._logger.info(".. preprocessing racetrack")
        self._preprocess_track()
        self._logger.info(".. optimization problem")
        self._optimize()
        self._logger.info(".. generating positional of raceling")
        self._generate_velocity_information()
        self._logger.info(".. generate velocity information")
        self._generate_velocity_information()

        return None

    def _generate_velocity_information(self) -> None:
        """
        Generates velocity information
        """
        self._vx_profile_opt = calc_vel_profile(
            ggv=self._ggv,
            ax_max_machines=self._engine_constraints,
            v_max=self._config.computation_config.general_config.vehicle_config.v_max,
            kappa=self._kappa_opt,
            el_lengths=self._el_lengths_opt_interp,
            closed=True,
            filt_window=self._config.computation_config.general_config.velocity_calc_config.velocity_profile_filter,
            dyn_model_exp=self._config.computation_config.general_config.velocity_calc_config.dyn_model_exp,
            drag_coeff=self._config.computation_config.general_config.vehicle_config.drag_coefficient,
            m_veh=self._config.computation_config.general_config.vehicle_config.mass
        )

        # calculate longitudinal acceleration profile
        self._vx_profile_opt_cl = np.append(self._vx_profile_opt, self._vx_profile_opt[0])
        self._ax_profile_opt = calc_ax_profile(
            vx_profile=self._vx_profile_opt_cl,
            el_lengths=self._el_lengths_opt_interp,
            eq_length_output=False
        )

        # calculate laptime
        self._t_profile_cl = calc_t_profile(
            vx_profile=self._vx_profile_opt,
            ax_profile=self._ax_profile_opt,
            el_lengths=self._el_lengths_opt_interp
        )

        self._logger.info("Estimated laptime: %.2fs" % self._t_profile_cl[-1])

    def _generate_positional_information(self) -> None:
        """
        Generates positional information of raceline.
        """
        self._dto.open_racetrack()
        self._raceline_interp, self._a_opt, self._coeffs_x_opt, self._coeffs_y_opt, \
            self._spline_inds_opt_interp, self._t_vals_opt_interp, self._s_points_opt_interp, \
            self._spline_lengths_opt, self._el_lengths_opt_interp = create_raceline(
                refline=self._dto.to_2d_np_array(),
                normvectors=self._normvec_normalized_interp,
                alpha=self._alpha_opt,
                stepsize_interp=self._config.computation_config.general_config.stepsize_config.stepsize_interp_after_opt
            )
        self._dto.close_racetrack()

        self._psi_vel_opt, self._kappa_opt = calc_head_curv_an(
            coeffs_x=self._coeffs_x_opt,
            coeffs_y=self._coeffs_y_opt,
            ind_spls=self._spline_inds_opt_interp,
            t_spls=self._t_vals_opt_interp
        )

    def _optimize(self) -> None:
        """
        Call optimization problem
        """
        self._alpha_opt, self._maximum_curvature_error = opt_min_curv(
            reftrack=self._dto,
            normvectors=self._normvec_normalized_interp,
            A=self._a_interp,
            kappa_bound=self._config.computation_config.general_config.vehicle_config.curvature_limit,
            w_veh=self._config.computation_config.optimization_config.opt_min_curvature_config.vehicle_width_opt,
            print_debug=self._config.execution_config.debug_config.debug,
            plot_debug=self._config.execution_config.debug_config.mincurv_curv_lin
        )

    def _preprocess_track(self) -> None:
        """
        Preprocesses track.
        """
        # close race track if not already done
        self._dto.close_racetrack()

        # liner interpolation
        interpolated_track: DtoFTM = LinearInterpolationLayer().linear_interpolate_racetrack(
            dto_racetrack=self._dto,
            interpol_stepsize=self._config.computation_config.general_config.stepsize_config.stepsize_preperation,
            return_new_instance=True
        )

        spline_track: DtoFTM = SplineApproxLayer().spline_approximation(
            dto_racetrack=self._dto,
            dto_racetrack_interpolated=interpolated_track,
            k_reg=self._config.computation_config.general_config.smoothing_config.k_reg,
            s_reg=self._config.computation_config.general_config.smoothing_config.s_reg,
            stepsize_reg=self._config.computation_config.general_config.stepsize_config.stepsize_regression,
            debug=self._config.execution_config.debug_config.debug,
        )

        # compute normals
        # TODO: move normals crossing horizon to config
        coeffs_x_interp, coeffs_y_interp, a_interp, normvec_normalized_interp = compute_normals_and_check_crosing(
            race_track=spline_track,
            normal_crossing_horizon=10
        )


        # inflate track
        if self._config.execution_config.import_config.min_track_width is not None:
            inflated_track: DtoFTM = WidthInflationLayer().inflate_width(
                dto_racetrack=spline_track,
                mininmum_track_width=self._config.execution_config.import_config.min_track_width,
                return_new_instance=False
            )
        else:
            inflated_track: DtoFTM = spline_track

        # set preprocessing values
        self._inflated_track: DtoFTM = inflated_track
        self._normvec_normalized_interp: np.ndarray = normvec_normalized_interp
        self._a_interp: np.ndarray = a_interp
        self._coeffs_x_interp: np.ndarray = coeffs_x_interp
        self._coeffs_y_interp: np.ndarray = coeffs_y_interp
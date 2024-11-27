import logging
from logging import Logger
import numpy as np

# own code
from commonroad_raceline_planner.configuration.ftm_config.ftm_config import FTMConfig
from commonroad_raceline_planner.planner.ftm_planner.optimization.opt_min_curv import (
    opt_min_curv,
)
from commonroad_raceline_planner.planner.base_planner import BaseRacelinePlanner
from commonroad_raceline_planner.planner.ftm_planner.velenis_vel_profile import (
    calc_vel_profile,
)
from commonroad_raceline_planner.raceline import RaceLine, RaceLineFactory
from commonroad_raceline_planner.planner.ftm_planner.track_processing.lin_interpol_layer import (
    LinearInterpolationLayer,
)
from commonroad_raceline_planner.planner.ftm_planner.track_processing.spline_approx_layer import (
    SplineApproxLayer,
)
from commonroad_raceline_planner.planner.ftm_planner.track_processing.width_inflation_layer import (
    WidthInflationLayer,
)
from commonroad_raceline_planner.racetrack import RaceTrack
from commonroad_raceline_planner.planner.ftm_planner.ftm_dto import (
    DtoFTM,
    DtoFTMFactory,
)
from commonroad_raceline_planner.planner.ftm_planner.track_processing.track_processing import (
    compute_normals_and_check_crosing,
)
from commonroad_raceline_planner.planner.ftm_planner.trajectory_planning.calc_ax_profile import (
    calc_ax_profile,
)
from commonroad_raceline_planner.planner.ftm_planner.trajectory_planning.calc_head_curv_an import (
    calc_head_curv_an,
)
from commonroad_raceline_planner.planner.ftm_planner.trajectory_planning.calc_t_profile import (
    calc_t_profile,
)
from commonroad_raceline_planner.planner.ftm_planner.trajectory_planning.create_raceline import (
    create_raceline,
)
from commonroad_raceline_planner.planner.ftm_planner.trajectory_planning.import_veh_dyn_info import (
    import_ggv_diagram,
    import_engine_constraints,
)
from commonroad_raceline_planner.planner.ftm_planner.trajectory_planning.validation import (
    check_traj,
)


# typing
from typing import Union


class MinimumCurvaturePlanner(BaseRacelinePlanner):

    def __init__(
        self, race_track: RaceTrack, config: FTMConfig, logger_level: int = logging.INFO
    ) -> None:
        """
        Minimum curvature planner. It uses convex optimization (QP) to minimize the (convex approximation of the) curvature.

        Paper: Heilmeier, A., Wischnewski, A., Hermansdorfer, L., Betz, J., Lienkamp, M., & Lohmann, B. (2020). Minimum curvature trajectory planning and control for an autonomous race car. Vehicle System Dynamics.

        Based on: AVS and FTM (2024): TUMFTM/global_racetrajectory_optimization. Available online at: https://github.com/TUMFTM/global_racetrajectory_optimization,
        AVS and FTM (2024): TUMFTM/trajectory_planning_helpers. Available online at: https://github.com/TUMFTM/trajectory_planning_helpers

        :param race_track: cr racetrack
        :param config: ftm_config
        :param logger_level: logger level, default info
        """
        super().__init__(race_track=race_track, config=config)

        # logger
        logging.basicConfig(level=logger_level)
        self._logger: Logger = logging.getLogger("FTMPlanner.MinCurv")
        self._logger.setLevel(logger_level)

        # import vehicle dynamics info
        self._ggv: np.ndarray = None
        self._engine_constraints: np.ndarray = None
        self.update_config(config=config)

        # Data transfer object
        self._dto: DtoFTM = DtoFTMFactory().generate_from_racetrack(race_track)

        # Preprocessing
        self._preprocessed_dto: DtoFTM = None
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

        # trajectory generation
        self._trajectory_opt: np.ndarray = None
        self.traj_race_cl: np.ndarray = None

        # raceline
        self._race_line: RaceLine = None

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

    @property
    def computed_race_line(self) -> Union[RaceLine, None]:
        """
        :return: computed race line or none if not computed
        """
        return self._race_line

    def update_config(self, config: FTMConfig) -> None:
        """
        Updates config
        :param config: FTM Config
        """
        self._config: FTMConfig = config
        self._ggv: np.ndarray = import_ggv_diagram(
            ggv_import_path=config.execution_config.filepath_config.ggv_file
        )
        self._engine_constraints: np.ndarray = import_engine_constraints(
            ax_max_machines_import_path=config.execution_config.filepath_config.ax_max_machines_file
        )
        self._reset_private_planning_members()

    def reset_planner(self) -> None:
        """
        Resets planner to before plan() was called
        """
        self._reset_private_planning_members()

    def plan(self) -> RaceLine:
        """
        Runs raceline planner
        :return: cr raceline object
        """
        self._logger.info(".. preprocessing racetrack")
        self._preprocess_track()
        self._logger.info(".. optimization problem")
        self._optimize()
        self._logger.info(".. compute positional information")
        self._compute_positional_information()
        self._logger.info(".. compute velocity information")
        self._compute_velocity_information()
        self._logger.info(".. generate trajectory")
        self._generate_raceline_object()
        self._logger.info(".. generate raceline object")

        return self._race_line

    def _generate_raceline_object(self) -> None:
        """
        validates generated trajectory and computes cr raceline object
        """
        # arrange data into one trajectory
        self._trajectory_opt = np.column_stack(
            (
                self._s_points_opt_interp,
                self._raceline_interp,
                self._psi_vel_opt,
                self._kappa_opt,
                self._vx_profile_opt,
                self._ax_profile_opt,
            )
        )
        spline_data_opt = np.column_stack(
            (self._spline_lengths_opt, self._coeffs_x_opt, self._coeffs_y_opt)
        )
        self._traj_race_cl = np.vstack(
            (self._trajectory_opt, self._trajectory_opt[0, :])
        )
        self._traj_race_cl[-1, 0] = np.sum(spline_data_opt[:, 0])  # set correct length

        # validate racetrack
        self._preprocessed_dto.open_racetrack()
        self._bound1, self._bound2 = check_traj(
            reftrack=self._preprocessed_dto,
            reftrack_normvec_normalized=self._normvec_normalized_interp,
            length_veh=self._config.computation_config.general_config.vehicle_config.length,
            width_veh=self._config.computation_config.general_config.vehicle_config.width,
            debug=self._config.execution_config.debug_config.debug,
            trajectory=self._trajectory_opt,
            ggv=self._ggv,
            ax_max_machines=self._engine_constraints,
            v_max=self._config.computation_config.general_config.vehicle_config.v_max,
            curvlim=self._config.computation_config.general_config.vehicle_config.curvature_limit,
            mass_veh=self._config.computation_config.general_config.vehicle_config.mass,
            dragcoeff=self._config.computation_config.general_config.vehicle_config.drag_coefficient,
        )
        self._preprocessed_dto.close_racetrack()

        # create reaceline
        self._race_line: RaceLine = RaceLineFactory().generate_raceline(
            length_per_point=self._s_points_opt_interp,
            points=self._raceline_interp,
            velocity_long_per_point=self._vx_profile_opt,
            acceleration_long_per_point=self._ax_profile_opt,
            curvature_per_point=self._kappa_opt,
            heading_per_point=self._psi_vel_opt,
            closed=True,
        )

    def _compute_velocity_information(self) -> None:
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
            m_veh=self._config.computation_config.general_config.vehicle_config.mass,
        )

        # calculate longitudinal acceleration profile
        self._vx_profile_opt_cl = np.append(
            self._vx_profile_opt, self._vx_profile_opt[0]
        )
        self._ax_profile_opt = calc_ax_profile(
            vx_profile=self._vx_profile_opt_cl,
            el_lengths=self._el_lengths_opt_interp,
            eq_length_output=False,
        )

        # calculate laptime
        self._t_profile_cl = calc_t_profile(
            vx_profile=self._vx_profile_opt,
            ax_profile=self._ax_profile_opt,
            el_lengths=self._el_lengths_opt_interp,
        )

        self._logger.info("Estimated laptime: %.2fs" % self._t_profile_cl[-1])

    def _compute_positional_information(self) -> None:
        """
        Computes positional information of raceline.
        """
        self._preprocessed_dto.open_racetrack()
        (
            self._raceline_interp,
            self._a_opt,
            self._coeffs_x_opt,
            self._coeffs_y_opt,
            self._spline_inds_opt_interp,
            self._t_vals_opt_interp,
            self._s_points_opt_interp,
            self._spline_lengths_opt,
            self._el_lengths_opt_interp,
        ) = create_raceline(
            refline=self._preprocessed_dto.to_2d_np_array(),
            normvectors=self._normvec_normalized_interp,
            alpha=self._alpha_opt,
            stepsize_interp=self._config.computation_config.general_config.stepsize_config.stepsize_interp_after_opt,
        )
        self._preprocessed_dto.close_racetrack()

        self._psi_vel_opt, self._kappa_opt = calc_head_curv_an(
            coeffs_x=self._coeffs_x_opt,
            coeffs_y=self._coeffs_y_opt,
            ind_spls=self._spline_inds_opt_interp,
            t_spls=self._t_vals_opt_interp,
        )

    def _optimize(self) -> None:
        """
        Call optimization problem
        """
        self._alpha_opt, self._maximum_curvature_error = opt_min_curv(
            reftrack=self._preprocessed_dto,
            normvectors=self._normvec_normalized_interp,
            A=self._a_interp,
            kappa_bound=self._config.computation_config.general_config.vehicle_config.curvature_limit,
            w_veh=self._config.computation_config.optimization_config.opt_min_curvature_config.vehicle_width_opt,
            print_debug=self._config.execution_config.debug_config.debug,
        )

    def _preprocess_track(self) -> None:
        """
        Preprocesses track.
        """
        # close race track if not already done
        self._dto.close_racetrack()

        # liner interpolation
        interpolated_track: (
            DtoFTM
        ) = LinearInterpolationLayer().linear_interpolate_racetrack(
            dto_racetrack=self._dto,
            interpol_stepsize=self._config.computation_config.general_config.stepsize_config.stepsize_preperation,
            return_new_instance=True,
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
        coeffs_x_interp, coeffs_y_interp, a_interp, normvec_normalized_interp = (
            compute_normals_and_check_crosing(
                race_track=spline_track, normal_crossing_horizon=10
            )
        )

        # inflate track
        if self._config.execution_config.min_track_width is not None:
            preprocessed_dto: DtoFTM = WidthInflationLayer().inflate_width(
                dto_racetrack=spline_track,
                mininmum_track_width=self._config.execution_config.min_track_width,
                return_new_instance=False,
            )
        else:
            preprocessed_dto: DtoFTM = spline_track

        # set preprocessing values
        self._preprocessed_dto: DtoFTM = preprocessed_dto
        self._normvec_normalized_interp: np.ndarray = normvec_normalized_interp
        self._a_interp: np.ndarray = a_interp
        self._coeffs_x_interp: np.ndarray = coeffs_x_interp
        self._coeffs_y_interp: np.ndarray = coeffs_y_interp

    def _reset_private_planning_members(self) -> None:
        """
        Resets all private members so missmatches are avoided
        """
        self._logger.info("Resetting planning")

        # Preprocessing
        self._preprocessed_dto: DtoFTM = None
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

        # trajectory generation
        self._trajectory_opt: np.ndarray = None
        self.traj_race_cl: np.ndarray = None

        # raceline
        self._race_line: RaceLine = None

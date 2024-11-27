import logging

# own code base
from commonroad_raceline_planner.racetrack import RaceTrack
from commonroad_raceline_planner.configuration.ftm_config.ftm_config import FTMConfig
from commonroad_raceline_planner.planner.ftm_planner.optimization.opt_shortest_path import (
    opt_shortest_path,
)
from commonroad_raceline_planner.planner.ftm_planner.ftm_mc_planner import (
    MinimumCurvaturePlanner,
)


# as this only differs from MinimumCurvature Planner in the optimization method, we inherit
# TODO: maybe make a parent class and inherit everything from there?
class ShortestPathPlanner(MinimumCurvaturePlanner):
    """
    FTM shortest path planner. It uses convex optimization (QP) to minimize the path length.

    Paper: Heilmeier, A., Wischnewski, A., Hermansdorfer, L., Betz, J., Lienkamp, M., & Lohmann, B. (2020). Minimum curvature trajectory planning and control for an autonomous race car. Vehicle System Dynamics.

    Based on: AVS and FTM (2024): TUMFTM/global_racetrajectory_optimization. Available online at: https://github.com/TUMFTM/global_racetrajectory_optimization,
    AVS and FTM (2024): TUMFTM/trajectory_planning_helpers. Available online at: https://github.com/TUMFTM/trajectory_planning_helpers
    """

    def __init__(
        self, race_track: RaceTrack, config: FTMConfig, logger_level: int = logging.INFO
    ) -> None:
        super().__init__(
            race_track=race_track, config=config, logger_level=logger_level
        )
        # overwrite logger
        self._logger = logging.getLogger("FTMPlanner.SP")
        self._logger.setLevel(logger_level)

    def _optimize(self) -> None:
        """
        Call optimization problem
        """
        self._alpha_opt = opt_shortest_path(
            reftrack=self._preprocessed_dto,
            normvectors=self._normvec_normalized_interp,
            vehicle_width=self._config.computation_config.optimization_config.opt_shortest_path_config.vehicle_width_opt,
            print_debug=self._config.execution_config.debug_config.debug,
        )

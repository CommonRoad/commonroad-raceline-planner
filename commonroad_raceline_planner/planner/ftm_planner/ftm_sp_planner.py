from commonroad_raceline_planner.optimization.opt_shortest_path import opt_shortest_path
from commonroad_raceline_planner.planner.ftm_planner.ftm_mc_planner import MinimumCurvaturePlanner


# as this only differs from MinimumCurvature Planner in the optimization method, we inherit
# TODO: maybe make a parent class and inherit everything from there?
class ShortestPathPlanner(MinimumCurvaturePlanner):
    """
    FTM shortest path planner from Heilmeier et al.
    """


    def _optimize(self) -> None:
        """
        Call optimization problem
        """
        self.alpha_opt = opt_shortest_path(
            reftrack=self._preprocessed_dto,
            normvectors=self._normvec_normalized_interp,
            vehicle_width=
            self._config.computation_config.optimization_config.opt_shortest_path_config.vehicle_width_opt,
            print_debug=self._config.execution_config.debug_config.debug
        )



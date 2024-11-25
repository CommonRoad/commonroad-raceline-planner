from typing import Union
from pathlib import Path

# commonroad
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario

# own package
from commonroad_raceline_planner.configuration.ftm_config.ftm_config import FTMConfig, FTMConfigFactory
from commonroad_raceline_planner.configuration.ftm_config.optimization_config import OptimizationType
from commonroad_raceline_planner.ractetrack import RaceTrackFactory, RaceTrack
from commonroad_raceline_planner.planner.ftm_planner.ftm_mc_planner import MinimumCurvaturePlanner
from commonroad_raceline_planner.planner.ftm_planner.ftm_sp_planner import ShortestPathPlanner
from commonroad_raceline_planner.raceline import RaceLine


# typing
from typing import Union

from commonroad_raceline_planner.util.visualization.visualize_on_racetrack import plot_trajectory_with_all_quantities
from commonroad_raceline_planner.util.visualization.visualize_over_arclength import plot_trajectory_over_arclength


def generate_ftm_raceline_from_cr_scenario(
        cr_scenario: Scenario,
        planning_problem: PlanningProblem,
        config_yml_path: Union[Path, str],
        ini_path: Union[Path, str],
        show_plot: bool = False
) -> RaceLine:
    """
    Generates raceline from commonroad scenario
    :param cr_scenario: cr scenario
    :param config_yml_path: .yml config path
    :param ini_path: .ini config path
    :return: cr raceline
    """
    # generate configs
    ftm_config: FTMConfig = FTMConfigFactory().generate_from_ini_and_config_file(
        path_to_ini=ini_path,
        path_to_config_yml=config_yml_path,
        optimization_type=OptimizationType.MINIMUM_CURVATURE
    )

    # import race track
    race_track = RaceTrackFactory().generate_racetrack_from_cr_scenario(
        lanelet_network=cr_scenario.lanelet_network,
        vehicle_width=ftm_config.computation_config.general_config.vehicle_config.width
    )

    # plan
    if ftm_config.execution_config.optimization_type == OptimizationType.MINIMUM_CURVATURE:
        mcp = MinimumCurvaturePlanner(
            config=ftm_config, race_track=race_track
        )
        raceline: RaceLine = mcp.plan()

    elif ftm_config.execution_config.optimization_type == OptimizationType.SHORTEST_PATH:
        spp = ShortestPathPlanner(
            config=ftm_config, race_track=race_track
        )
        raceline: RaceLine = spp.plan()
    else:
        raise NotImplementedError(f'Planner {ftm_config.execution_config.optimization_type} not implemented')

    if show_plot:
        plot_trajectory_with_all_quantities(
            race_line=raceline,
            lanelet_network=cr_scenario.lanelet_network,
            planning_problem=planning_problem
        )

        plot_trajectory_over_arclength(
            race_line=raceline
        )

    return raceline



def generate_ftm_raceline_from_cvs_scenario(
        path_to_csv: Union[Path, str],
        racetrack: RaceTrack,
        planning_problem: PlanningProblem,
        config_yml_path: Union[Path, str],
        ini_path: Union[Path, str],
        show_plot: bool = False
) -> RaceLine:
    # generate configs
    ftm_config: FTMConfig = FTMConfigFactory().generate_from_ini_and_config_file(
        path_to_ini=ini_path,
        path_to_config_yml=config_yml_path,
        optimization_type=OptimizationType.MINIMUM_CURVATURE
    )

    # import race track
    race_track = RaceTrackFactory().generate_racetrack_from_csv(
        file_path=path_to_csv,
        vehicle_width=ftm_config.computation_config.general_config.vehicle_config.width
    )

    # plan
    if ftm_config.execution_config.optimization_type == OptimizationType.MINIMUM_CURVATURE:
        mcp = MinimumCurvaturePlanner(
            config=ftm_config, race_track=race_track
        )
        raceline: RaceLine = mcp.plan()

    elif ftm_config.execution_config.optimization_type == OptimizationType.SHORTEST_PATH:
        spp = ShortestPathPlanner(
            config=ftm_config, race_track=race_track
        )
        raceline: RaceLine = spp.plan()
    else:
        raise NotImplementedError(f'Planner {ftm_config.execution_config.optimization_type} not implemented')

    if show_plot:
        plot_trajectory_with_all_quantities(
            race_line=raceline,
            lanelet_network=cr_scenario.lanelet_network,
            planning_problem=planning_problem
        )

        plot_trajectory_over_arclength(
            race_line=raceline
        )

    return raceline



















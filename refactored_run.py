from typing import Union
from pathlib import Path

# commonroad
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblem

# own package
from commonroad_raceline_planner.configuration.ftm_config.ftm_config import FTMConfig, FTMConfigFactory
from commonroad_raceline_planner.configuration.ftm_config.optimization_config import OptimizationType
from commonroad_raceline_planner.ractetrack import  RaceTrackFactory
from commonroad_raceline_planner.planner.ftm_planner.ftm_mc_planner import MinimumCurvaturePlanner
from commonroad_raceline_planner.planner.ftm_planner.ftm_sp_planner import ShortestPathPlanner
from commonroad_raceline_planner.raceline import RaceLine
from commonroad_raceline_planner.util.visualization.visualize_on_racetrack import plot_trajectory_with_all_quantities
from commonroad_raceline_planner.util.visualization.visualize_over_arclength import plot_trajectory_over_arclength


def main(
        cr_path: Union[str, Path],
        config_path: Union[str, Path],
        ini_path: Union[str, Path],
) -> None:
    # Commonroad imports
    scenario, planning_problem_set = CommonRoadFileReader(cr_path).open()
    planning_problem: PlanningProblem = list(planning_problem_set.planning_problem_dict.values())[0]

    # generate configs
    ftm_config: FTMConfig = FTMConfigFactory().generate_from_ini_and_config_file(
        path_to_ini=ini_path,
        path_to_config_yml=config_path,
        optimization_type=OptimizationType.MINIMUM_CURVATURE
    )

    # import race track
    race_track = RaceTrackFactory().generate_racetrack_from_cr_scenario(
        lanelet_network=scenario.lanelet_network,
        planning_problem=planning_problem,
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
        raise NotImplementedError(f'Planner not implemented')

    # export data
    raceline.export_trajectory_to_csv_file(
        export_path=ftm_config.execution_config.filepath_config.traj_race_export,
        ggv_file_path=ftm_config.execution_config.filepath_config.ggv_file
    )

    plot_trajectory_with_all_quantities(
        race_line=raceline,
        lanelet_network=scenario.lanelet_network,
        planning_problem=planning_problem
    )

    plot_trajectory_over_arclength(
        race_line=raceline
    )


if __name__ == "__main__":
    config_path: Path = Path(__file__).parents[0] / "configurations/race_line_planner_config.yaml"
    cr_path = "/home/tmasc/projects/cr-raceline/commonroad-raceline-planner/inputs/tracks/XML_maps/ZAM_realrounded-1_1_T-1.xml"
    ini_path = "/home/tmasc/projects/cr-raceline/commonroad-raceline-planner/inputs/params/racecar.ini"
    main(
        config_path=config_path,
        cr_path=cr_path,
        ini_path=ini_path,
    )

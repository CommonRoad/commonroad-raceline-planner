from pathlib import Path

# commonroad
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario

# own package
from commonroad_raceline_planner.configuration.ftm_config.ftm_config import (
    FTMConfig,
    FTMConfigFactory,
)
from commonroad_raceline_planner.configuration.ftm_config.optimization_config import (
    OptimizationType,
)
from commonroad_raceline_planner.racetrack import RaceTrackFactory
from commonroad_raceline_planner.planner.ftm_planner.ftm_mc_planner import (
    MinimumCurvaturePlanner,
)
from commonroad_raceline_planner.planner.ftm_planner.ftm_sp_planner import (
    ShortestPathPlanner,
)
from commonroad_raceline_planner.raceline import RaceLine
from commonroad_raceline_planner.utils.visualization.visualize_on_racetrack import (
    plot_trajectory_with_all_quantities,
)
from commonroad_raceline_planner.utils.visualization.visualize_over_arclength import (
    plot_trajectory_over_arclength,
)

# typing
from typing import Union, Optional


def generate_ftm_shortest_path_raceline_from_cr(
    cr_scenario: Scenario,
    planning_problem: PlanningProblem,
    ini_path: Union[str, Path],
    ggv_file: Union[str, Path],
    ax_max_machines_file: Union[str, Path],
    traj_race_export: Optional[Union[str, Path]] = None,
    velocity_profile_export: Optional[Union[str, Path]] = None,
    min_track_width: Optional[float] = None,
    show_plot: bool = False,
) -> RaceLine:
    """
    Generate cr raceline with FTM shortest path algorithm.

    Paper: Heilmeier, A., Wischnewski, A., Hermansdorfer, L., Betz, J., Lienkamp, M., & Lohmann, B. (2020). Minimum curvature trajectory planning and control for an autonomous race car. Vehicle System Dynamics.

    Based on: AVS and FTM (2024): TUMFTM/global_racetrajectory_optimization. Available online at: https://github.com/TUMFTM/global_racetrajectory_optimization,
    AVS and FTM (2024): TUMFTM/trajectory_planning_helpers. Available online at: https://github.com/TUMFTM/trajectory_planning_helpers

    :param cr_scenario: cr scenario
    :param planning_problem: cr planning problem
    :param ini_path: path to .ini file
    :param ggv_file: path to ggv file
    :param ax_max_machines_file: path to engine constraints file
    :param traj_race_export: file path for trajectory export, may be none
    :param velocity_profile_export: fale path for velocity profile export, may be none
    :param min_track_width: min additional track width, default none
    :param show_plot: shows plots if true
    :return: cr raceline
    """
    opt_type = OptimizationType.SHORTEST_PATH
    return generate_ftm_raceline_from_cr_scenario(
        cr_scenario=cr_scenario,
        planning_problem=planning_problem,
        ini_path=ini_path,
        ggv_file=ggv_file,
        ax_max_machines_file=ax_max_machines_file,
        opt_type=opt_type,
        traj_race_export=traj_race_export,
        velocity_profile_export=velocity_profile_export,
        min_track_width=min_track_width,
        show_plot=show_plot,
    )


def generate_ftm_minimum_curvature_raceline_from_cr(
    cr_scenario: Scenario,
    planning_problem: PlanningProblem,
    ini_path: Union[str, Path],
    ggv_file: Union[str, Path],
    ax_max_machines_file: Union[str, Path],
    traj_race_export: Optional[Union[str, Path]] = None,
    velocity_profile_export: Optional[Union[str, Path]] = None,
    min_track_width: Optional[float] = None,
    show_plot: bool = False,
) -> RaceLine:
    """
    Generate cr raceline with FTM minimum curvature algorithm

    Paper: Heilmeier, A., Wischnewski, A., Hermansdorfer, L., Betz, J., Lienkamp, M., & Lohmann, B. (2020). Minimum curvature trajectory planning and control for an autonomous race car. Vehicle System Dynamics.

    Based on: AVS and FTM (2024): TUMFTM/global_racetrajectory_optimization. Available online at: https://github.com/TUMFTM/global_racetrajectory_optimization,
    AVS and FTM (2024): TUMFTM/trajectory_planning_helpers. Available online at: https://github.com/TUMFTM/trajectory_planning_helpers

    :param cr_scenario: cr scenario
    :param planning_problem: cr planning problem
    :param ini_path: path to .ini file
    :param ggv_file: path to ggv file
    :param ax_max_machines_file: path to engine constraints file
    :param traj_race_export: file path for trajectory export, may be none
    :param velocity_profile_export: fale path for velocity profile export, may be none
    :param min_track_width: min additional track width, default none
    :param show_plot: shows plots if true
    :return: cr raceline
    """
    opt_type = OptimizationType.MINIMUM_CURVATURE
    return generate_ftm_raceline_from_cr_scenario(
        cr_scenario=cr_scenario,
        planning_problem=planning_problem,
        ini_path=ini_path,
        ggv_file=ggv_file,
        ax_max_machines_file=ax_max_machines_file,
        opt_type=opt_type,
        traj_race_export=traj_race_export,
        velocity_profile_export=velocity_profile_export,
        min_track_width=min_track_width,
        show_plot=show_plot,
    )


def generate_ftm_raceline_from_cr_scenario(
    cr_scenario: Scenario,
    planning_problem: PlanningProblem,
    ini_path: Union[str, Path],
    ggv_file: Union[str, Path],
    ax_max_machines_file: Union[str, Path],
    opt_type: OptimizationType,
    traj_race_export: Optional[Union[str, Path]] = None,
    velocity_profile_export: Optional[Union[str, Path]] = None,
    min_track_width: Optional[float] = None,
    show_plot: bool = False,
) -> RaceLine:
    """
    Generate cr raceline with selectable FTM planner

    Paper: Heilmeier, A., Wischnewski, A., Hermansdorfer, L., Betz, J., Lienkamp, M., & Lohmann, B. (2020). Minimum curvature trajectory planning and control for an autonomous race car. Vehicle System Dynamics.

    Based on: AVS and FTM (2024): TUMFTM/global_racetrajectory_optimization. Available online at: https://github.com/TUMFTM/global_racetrajectory_optimization,
    AVS and FTM (2024): TUMFTM/trajectory_planning_helpers. Available online at: https://github.com/TUMFTM/trajectory_planning_helpers

    :param cr_scenario: cr scenario
    :param planning_problem: cr planning problem
    :param opt_type: optimization type
    :param ini_path: path to .ini file
    :param ggv_file: path to ggv file
    :param ax_max_machines_file: path to engine constraints file
    :param traj_race_export: file path for trajectory export, may be none
    :param velocity_profile_export: fale path for velocity profile export, may be none
    :param min_track_width: min additional track width, default none
    :param show_plot: shows plots if true
    :return: cr raceline
    """
    # generate configs
    ftm_config: FTMConfig = FTMConfigFactory().generate_from_files(
        path_to_ini=ini_path,
        ggv_file=ggv_file,
        ax_max_machines_file=ax_max_machines_file,
        optimization_type=OptimizationType.MINIMUM_CURVATURE,
        min_track_width=min_track_width,
    )

    # import race track
    race_track = RaceTrackFactory().generate_racetrack_from_cr_scenario(
        lanelet_network=cr_scenario.lanelet_network,
        planning_problem=planning_problem,
        vehicle_width=ftm_config.computation_config.general_config.vehicle_config.width,
    )

    # plan
    if opt_type == OptimizationType.MINIMUM_CURVATURE:
        mcp = MinimumCurvaturePlanner(config=ftm_config, race_track=race_track)
        raceline: RaceLine = mcp.plan()

    elif opt_type == OptimizationType.SHORTEST_PATH:
        spp = ShortestPathPlanner(config=ftm_config, race_track=race_track)
        raceline: RaceLine = spp.plan()
    else:
        raise NotImplementedError(f"Planner {opt_type} not implemented")

    # export data
    if traj_race_export is not None and ggv_file is not None:
        raceline.export_trajectory_to_csv_file(
            export_path=traj_race_export, ggv_file_path=velocity_profile_export
        )

    if show_plot:
        plot_trajectory_with_all_quantities(
            race_line=raceline,
            lanelet_network=cr_scenario.lanelet_network,
            planning_problem=planning_problem,
        )

        plot_trajectory_over_arclength(race_line=raceline)

    return raceline

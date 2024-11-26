from typing import Union, Optional
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
        ini_path: Union[str, Path],
        ggv_file: Union[str, Path],
        ax_max_machines_file: Union[str, Path],
        traj_race_export: Optional[Union[str, Path]] = None,
        velocity_profile_export: Optional[Union[str, Path]] = None,
        opt_type: OptimizationType = OptimizationType.SHORTEST_PATH,
        min_track_width: Optional[float] = None
) -> None:
    # Commonroad imports
    scenario, planning_problem_set = CommonRoadFileReader(cr_path).open()
    planning_problem: PlanningProblem = list(planning_problem_set.planning_problem_dict.values())[0]

    # generate configs
    ftm_config: FTMConfig = FTMConfigFactory().generate_from_files(
        path_to_ini=ini_path,
        ggv_file=ggv_file,
        ax_max_machines_file=ax_max_machines_file,
        optimization_type=OptimizationType.MINIMUM_CURVATURE,
        min_track_width=min_track_width
    )

    # import race track
    race_track = RaceTrackFactory().generate_racetrack_from_cr_scenario(
        lanelet_network=scenario.lanelet_network,
        planning_problem=planning_problem,
        vehicle_width=ftm_config.computation_config.general_config.vehicle_config.width
    )


    # plan
    if opt_type == OptimizationType.MINIMUM_CURVATURE:
        mcp = MinimumCurvaturePlanner(
            config=ftm_config, race_track=race_track
        )
        raceline: RaceLine = mcp.plan()

    elif opt_type == OptimizationType.SHORTEST_PATH:
        spp = ShortestPathPlanner(
            config=ftm_config, race_track=race_track
        )
        raceline: RaceLine = spp.plan()
    else:
        raise NotImplementedError(f'Planner {opt_type} not implemented')

    # export data
    if traj_race_export is not None and ggv_file is not None:
        raceline.export_trajectory_to_csv_file(
            export_path=traj_race_export,
            ggv_file_path=velocity_profile_export
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
    cr_path = "/scenarios/tracks/XML_maps/tests/ZAM_realrounded-1_1_T-1.xml"
    ini_path = "/home/tmasc/projects/cr-raceline/commonroad-raceline-planner/scenarios/params/racecar.ini"
    ggv_file = "/home/tmasc/projects/cr-raceline/commonroad-raceline-planner/scenarios/veh_dyn_info/ggv.csv"
    ax_max_machines_file = "/home/tmasc/projects/cr-raceline/commonroad-raceline-planner/scenarios/veh_dyn_info/ax_max_machines.csv"
    traj_race_export = "/home/tmasc/projects/cr-raceline/commonroad-raceline-planner/outputs/traj_race_cl.csv"
    velocity_profile_export = "/home/tmasc/projects/cr-raceline/commonroad-raceline-planner/outputs/velocity_profile.json"
    opt_type: OptimizationType = OptimizationType.MINIMUM_CURVATURE
    main(
        cr_path=cr_path,
        ini_path=ini_path,
        ggv_file=ggv_file,
        ax_max_machines_file=ax_max_machines_file,
        traj_race_export=traj_race_export,
        velocity_profile_export=velocity_profile_export,
        opt_type=opt_type,
    )

import unittest
from pathlib import Path
import os

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblem

from commonroad_raceline_planner.configuration.ftm_config.optimization_config import OptimizationType
from commonroad_raceline_planner.raceline import RaceLine
from commonroad_raceline_planner.raceline_fast_api import (
    generate_ftm_minimum_curvature_raceline_from_cr,
    generate_ftm_shortest_path_raceline_from_cr
)


class TestCRMincurv(unittest.TestCase):
    """
    Test Min Curv on CommonRoad
    """

    def test_cr_mincuv(self):
        path_scenarios = Path(__file__).parents[1] / "scenarios"

        cr_paths = path_scenarios / "tracks/XML_maps/tests"
        cr_files = os.listdir(cr_paths)

        ini_path = path_scenarios / "params/racecar.ini"
        ggv_file = path_scenarios / "veh_dyn_info/ggv.csv"
        ax_max_machines_file = path_scenarios / "veh_dyn_info/ax_max_machines.csv"


        for filename in cr_files:
            with self.subTest(msg=f"Testing scenario: {filename} with shortest path fapi", filename=filename):
                scenario, planning_problem_set = CommonRoadFileReader(cr_paths / filename).open()
                planning_problem: PlanningProblem = list(planning_problem_set.planning_problem_dict.values())[0]

                raceline: RaceLine = generate_ftm_shortest_path_raceline_from_cr(
                    cr_scenario=scenario,
                    planning_problem=planning_problem,
                    ini_path=ini_path,
                    ggv_file=ggv_file,
                    ax_max_machines_file=ax_max_machines_file,
                    traj_race_export=None,
                    velocity_profile_export= None,
                    min_track_width=None,
                    show_plot=True
                )

            with self.subTest(msg=f"Testing scenario: {filename} with minmum curvature fapi", filename=filename):
                raceline: RaceLine = generate_ftm_minimum_curvature_raceline_from_cr(
                    cr_scenario=scenario,
                    planning_problem=planning_problem,
                    ini_path=ini_path,
                    ggv_file=ggv_file,
                    ax_max_machines_file=ax_max_machines_file,
                    traj_race_export=None,
                    velocity_profile_export=None,
                    min_track_width=None,
                    show_plot=True
                )

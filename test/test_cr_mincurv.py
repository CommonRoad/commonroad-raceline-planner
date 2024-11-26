import unittest
from pathlib import Path
import os

from commonroad_raceline_planner.configuration.ftm_config.optimization_config import OptimizationType
from run_planner_example import main


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
        opt_type: OptimizationType = OptimizationType.MINIMUM_CURVATURE


        for filename in cr_files:
            with self.subTest(msg=f"Testing scenario: {filename} with minmum curvature", filename=filename):
                main(
                    cr_path=cr_paths / filename,
                    ini_path=ini_path,
                    ggv_file=ggv_file,
                    ax_max_machines_file=ax_max_machines_file,
                    opt_type=opt_type
                )








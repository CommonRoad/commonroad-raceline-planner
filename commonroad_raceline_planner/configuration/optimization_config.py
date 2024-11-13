import enum
import os.path
from dataclasses import dataclass
from pathlib import Path
from configparser import ConfigParser
import json

# own code base
from commonroad_raceline_planner.configuration.base_config import BaseConfigFactory

# typing
from typing import Union

@enum.unique
class OptimizationType(enum.Enum):
    SHORTEST_PATH = "shortest_path"
    MINIMUM_CURVATURE = "mincurv"
    MINIMUM_LAPTIME = "mintime"


#----- Optimization Config
@dataclass
class OptShortestPathConfig:
    vehicle_width_opt: float

@dataclass
class OptMinimumCurvatureConfig:
    vehicle_width_opt: float
    min_iterations: int
    allowed_curvature_error: float


@dataclass
class OptimizationConfig:
    opt_shortest_path_config: Union[OptShortestPathConfig, None] = None
    opt_min_curvature_config: Union[OptMinimumCurvatureConfig, None] = None

    def __post_init__(self):
        if self.opt_shortest_path_config is None and self.opt_min_curvature_config is None:
            raise ValueError('No _execution_config not set')


# - Factory
class OptimizationConfigFactory(BaseConfigFactory):
    """
    Generates optimization _execution_config from .ini file
    """

    def generate_from_racecar_ini(
            self,
            path_to_racecar_ini: Union[Path, str],
            optimization_type: OptimizationType
    ) -> OptimizationConfig:
        """
        Generates Optimization Config from racecar ini
        :param path_to_racecar_ini: absolut path to racecar ini
        :param optimization_type: optimization type
        :return: optimization _execution_config object
        """

        # load _execution_config
        self._load_ini(path_to_racecar_ini)

        opt_shortest_path_config: Union[OptShortestPathConfig, None] = None
        opt_min_curvature_config: Union[OptMinimumCurvatureConfig, None] = None

        # transform to sub-data classes
        if optimization_type == OptimizationType.SHORTEST_PATH:
            conf = json.loads(
                self._parser.get(
                    section='OPTIMIZATION_OPTIONS',
                    option='optim_opts_shortest_path'
                )
            )
            opt_shortest_path_config = OptShortestPathConfig(
                vehicle_width_opt=conf["width_opt"]
            )
        elif optimization_type == OptimizationType.MINIMUM_CURVATURE:
            conf = json.loads(
                self._parser.get(
                    section='OPTIMIZATION_OPTIONS',
                    option='optim_opts_mincurv'
                )
            )
            opt_min_curvature_config = OptMinimumCurvatureConfig(
                vehicle_width_opt=conf["width_opt"],
                min_iterations=conf["iqp_iters_min"],
                allowed_curvature_error=conf["iqp_curverror_allowed"]
            )
        else:
            raise ValueError(f"Optimization type {optimization_type} not a valid option.")

        return OptimizationConfig(
            opt_shortest_path_config=opt_shortest_path_config,
            opt_min_curvature_config=opt_min_curvature_config
        )

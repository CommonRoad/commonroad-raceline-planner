from dataclasses import dataclass
from pathlib import Path

from commonroad_raceline_planner.configuration.base_config import BaseConfigFactory
from commonroad_raceline_planner.configuration.ftm_config.computation_config import ComputationConfig, \
    ComputationConfigFactory
from commonroad_raceline_planner.configuration.ftm_config.execution_config import ExecutionConfig, \
    ExecutionConfigFactory

# typing
from typing import Union

from commonroad_raceline_planner.configuration.ftm_config.optimization_config import OptimizationType, \
    OptimizationConfigFactory


@dataclass
class FTMConfig:
    """
    Config for both Heilmeier planners
    :param execution_config: execution config
    :param compuation_config: computation config
    """
    execution_config: ExecutionConfig
    computation_config: ComputationConfig


class FTMConfigFactory(BaseConfigFactory):
    """
    Factory for FTM Config
    """
    @staticmethod
    def generate_from_ini_and_config_file(
            path_to_ini: Union[str, Path],
            path_to_config_yml: Union[str, Path],
            optimization_type: OptimizationType = OptimizationType.MINIMUM_CURVATURE
    ) -> FTMConfig:
        """
        Generates Confits for both planners from Heilmeier et al. from ini and config file
        :param path_to_ini: path .ini config file
        :param path_to_config_yml: path to .yml config file
        :param optimization_type: optimization type. Choose between minimum curvature and shortest path
        :return: ftm config
        """
        return FTMConfig(
            computation_config=ComputationConfigFactory().generate_from_racecar_ini(
                path_to_racecar_ini=path_to_ini,
                optimization_type=optimization_type
            ),
            execution_config=ExecutionConfigFactory().generate_from_yml(path_to_yml=path_to_config_yml)
        )





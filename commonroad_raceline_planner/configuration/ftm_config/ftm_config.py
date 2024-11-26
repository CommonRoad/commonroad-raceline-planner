from dataclasses import dataclass
from pathlib import Path

# own code base
from commonroad_raceline_planner.configuration.base_config import BaseConfigFactory
from commonroad_raceline_planner.configuration.ftm_config.computation_config import (
    ComputationConfig,
    ComputationConfigFactory,
)
from commonroad_raceline_planner.configuration.ftm_config.execution_config import (
    ExecutionConfig,
    ExecutionConfigFactory,
)
from commonroad_raceline_planner.configuration.ftm_config.optimization_config import (
    OptimizationType,
)

# typing
from typing import Union, Optional


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
    def generate_from_files(
        path_to_ini: Union[str, Path],
        ax_max_machines_file: Union[Path, str],
        ggv_file: Union[Path, str],
        min_track_width: Optional[float] = None,
        optimization_type: OptimizationType = OptimizationType.MINIMUM_CURVATURE,
        debug: bool = False,
    ) -> FTMConfig:
        """
        Generates ftm config
        :param path_to_ini: path to .ini file
        :param ax_max_machines_file: path to engine constraints file
        :param ggv_file: path ggv file
        :param min_track_width: optional -> minimum track width to be considered
        :param optimization_type: optimization type (default minimum curvature)
        :param debug: (default false)
        :return: cr ftm config
        """
        return FTMConfig(
            computation_config=ComputationConfigFactory().generate_from_racecar_ini(
                path_to_racecar_ini=path_to_ini,
            ),
            execution_config=ExecutionConfigFactory().generate_exec_config(
                veh_params_file=path_to_ini,
                ax_max_machines_file=ax_max_machines_file,
                ggv_file=ggv_file,
                min_track_width=min_track_width,
                optimization_type=optimization_type,
                debug=debug,
            ),
        )

from dataclasses import dataclass
from pathlib import Path

# typing
from typing import Optional, Union

from commonroad_raceline_planner.configuration.base_config import BaseConfigFactory
from commonroad_raceline_planner.configuration.ftm_config.optimization_config import (
    OptimizationType,
)


@dataclass
class DebugConfig:  # Debug options
    debug: bool


@dataclass
class FilePathConfig:
    veh_params_file: str
    ax_max_machines_file: str
    ggv_file: str


@dataclass
class ExecutionConfig:
    debug_config: DebugConfig
    filepath_config: FilePathConfig
    optimization_type: OptimizationType
    min_track_width: Optional[float]


class ExecutionConfigFactory(BaseConfigFactory):
    """
    Generates execution _execution_config
    """

    def generate_exec_config(
        self,
        veh_params_file: Union[Path, str],
        ax_max_machines_file: Union[Path, str],
        ggv_file: Union[Path, str],
        optimization_type: OptimizationType,
        min_track_width: Optional[float] = None,
        debug: bool = False,
    ) -> ExecutionConfig:
        """
        Generates Optimization Config from racecar ini
        :param path_to_racecar_ini: absolut path to racecar ini
        :param optimization_type: optimization type
        :return: optimization _execution_config object
        """

        # debug _execution_config
        debug_config = DebugConfig(debug=debug)

        # file path opts
        filepath_config = FilePathConfig(
            veh_params_file=veh_params_file,
            ggv_file=ggv_file,
            ax_max_machines_file=ax_max_machines_file,
        )

        return ExecutionConfig(
            debug_config=debug_config,
            filepath_config=filepath_config,
            optimization_type=optimization_type,
            min_track_width=min_track_width,
        )

from dataclasses import dataclass
from pathlib import Path

# own code base
from commonroad_raceline_planner.configuration.base_config import BaseConfigFactory
from commonroad_raceline_planner.configuration.ftm_config.optimization_config import (
    OptimizationConfigFactory,
    OptimizationConfig,
)
from commonroad_raceline_planner.configuration.ftm_config.general_config import (
    GeneralConfigFactory,
    GeneralConfig,
)

# typing
from typing import Union


@dataclass
class ComputationConfig:
    # TODO: Find better name
    general_config: GeneralConfig
    optimization_config: OptimizationConfig


# - Factory
class ComputationConfigFactory(BaseConfigFactory):
    """
    Generates overall _execution_config from .ini file
    """

    def generate_from_racecar_ini(
        self,
        path_to_racecar_ini: Union[Path, str],
    ) -> ComputationConfig:

        # TODO: remove currently doubled sanity check
        # sanity check
        if not self._sanity_check_ini(file_path=path_to_racecar_ini):
            raise FileNotFoundError(
                f"Did not find .ini file at absolute path {path_to_racecar_ini}"
            )

        general_config: (
            GeneralConfig
        ) = GeneralConfigFactory().generate_from_racecar_ini(
            path_to_racecar_ini=path_to_racecar_ini
        )

        optimization_config: (
            OptimizationConfig
        ) = OptimizationConfigFactory().generate_from_racecar_ini(
            path_to_racecar_ini=path_to_racecar_ini
        )

        return ComputationConfig(
            general_config=general_config, optimization_config=optimization_config
        )

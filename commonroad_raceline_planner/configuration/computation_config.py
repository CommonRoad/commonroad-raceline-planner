from dataclasses import dataclass
from pathlib import Path

# own code base
from commonroad_raceline_planner.configuration.base_config import BaseConfigFactory
from commonroad_raceline_planner.configuration.optimization_config import (
    OptimizationConfigFactory,
    OptimizationType,
    OptimizationConfig
)
from commonroad_raceline_planner.configuration.general_config import (
    GeneralConfigFactory,
    GeneralConfig
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
    Generates overall config from .ini file
    """

    def generate_from_racecar_ini(
            self,
            path_to_racecar_ini: Union[Path, str],
            optimization_type: OptimizationType = OptimizationType.MINIMUM_CURVATURE
    ) -> ComputationConfig:

        # TODO: remove currently doubled sanity check
        # sanity check
        if not self._sanity_check_ini(path_to_racecar_ini=path_to_racecar_ini):
            raise FileNotFoundError(f'Did not find .ini file at absolute path {path_to_racecar_ini}')

        general_config: GeneralConfig = GeneralConfigFactory().generate_from_racecar_ini(
            path_to_racecar_ini=path_to_racecar_ini
        )

        # TODO: include optimization type read out
        optimization_config: OptimizationConfig = OptimizationConfigFactory().generate_from_racecar_ini(
            path_to_racecar_ini=path_to_racecar_ini,
            optimization_type=optimization_type
        )

        return ComputationConfig(
            general_config=general_config,
            optimization_config=optimization_config
        )










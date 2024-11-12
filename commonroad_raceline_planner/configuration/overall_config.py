from dataclasses import dataclass


# typing
from general_config import GeneralConfig
from optimization_config import OptimizationConfig


@dataclass
class OverallConfig:
    # TODO: Find better name
    general_config: GeneralConfig
    optimization_config: OptimizationConfig










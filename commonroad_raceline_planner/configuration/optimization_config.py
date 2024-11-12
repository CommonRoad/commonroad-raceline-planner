from dataclasses import dataclass
from pathlib import Path
from configparser import ConfigParser

# typing
from typing import Union

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
            raise ValueError('Config not set')






class OptimizationConfigFactory:
    """
    Generates optimization config from .ini file
    """
    def __init__(self):
        self.parser = ConfigParser()


    def generate_from_racecar_ini(
            self,
            path_to_racecar_ini: Union[Path, str]
    ) -> OptimizationConfig:

        # pars .init file
        self.parser.read(path_to_racecar_ini)

        # Add attributes to dict
        add_to_dict(pars, "ggv_file", json.loads(parser.get('GENERAL_OPTIONS', 'ggv_file')))
        add_to_dict(pars, "ax_max_machines_file", json.loads(parser.get('GENERAL_OPTIONS', 'ax_max_machines_file')))
        add_to_dict(pars, "stepsize_opts", json.loads(parser.get('GENERAL_OPTIONS', 'stepsize_opts')))
        add_to_dict(pars, "reg_smooth_opts", json.loads(parser.get('GENERAL_OPTIONS', 'reg_smooth_opts')))
        add_to_dict(pars, "veh_params", json.loads(parser.get('GENERAL_OPTIONS', 'veh_params')))
        add_to_dict(pars, "vel_calc_opts", json.loads(parser.get('GENERAL_OPTIONS', 'vel_calc_opts')))

        if config.opt_type == 'shortest_path':
            pars["optim_opts"] = json.loads(parser.get('OPTIMIZATION_OPTIONS', 'optim_opts_shortest_path'))
        elif config.opt_type == 'mincurv':
            pars["optim_opts"] = json.loads(parser.get('OPTIMIZATION_OPTIONS', 'optim_opts_mincurv'))
        elif config.opt_type == 'mintime':
            pars["curv_calc_opts"] = json.loads(parser.get('GENERAL_OPTIONS', 'curv_calc_opts'))
            pars["optim_opts"] = json.loads(parser.get('OPTIMIZATION_OPTIONS', 'optim_opts_mintime'))
            pars["vehicle_params_mintime"] = json.loads(parser.get('OPTIMIZATION_OPTIONS', 'vehicle_params_mintime'))
            pars["tire_params_mintime"] = json.loads(parser.get('OPTIMIZATION_OPTIONS', 'tire_params_mintime'))
            pars["pwr_params_mintime"] = json.loads(parser.get('OPTIMIZATION_OPTIONS', 'pwr_params_mintime'))

            # modification of mintime options/parameters
            pars["optim_opts"]["var_friction"] = config.mintime_opts["var_friction"]
            pars["optim_opts"]["warm_start"] = config.mintime_opts["warm_start"]
            pars["vehicle_params_mintime"]["wheelbase"] = (pars["vehicle_params_mintime"]["wheelbase_front"]
                                                           + pars["vehicle_params_mintime"]["wheelbase_rear"])

        if not (config.opt_type == 'mintime' and not config.mintime_opts['recalc_vel_profile_by_tph']):
            add_to_dict(file_paths, "ggv_file",
                        os.path.join(file_paths["module"], "inputs", "veh_dyn_info", pars["ggv_file"]))
            add_to_dict(file_paths, "ax_max_machines_file",
                        os.path.join(file_paths["module"], "inputs", "veh_dyn_info", pars["ax_max_machines_file"]))

        return pars
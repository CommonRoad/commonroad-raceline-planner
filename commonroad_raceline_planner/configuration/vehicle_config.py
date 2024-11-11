import configparser
import copy
import json
import os
from commonroad_raceline_planner.configuration.race_line_config import RaceLinePlannerConfiguration
from commonroad_raceline_planner.util.aziz_helpers.helper_functions import add_to_dict


# typing
from typing import Dict

def setup_vehicle_parameters(
    config: RaceLinePlannerConfiguration,
) -> Dict:
    """
    Set up the vehicle parameters by reading from the configuration file.
    """
    file_paths = config.file_paths

    parser = configparser.ConfigParser()
    pars = {}

    parser.read(os.path.join(file_paths["module"], file_paths["veh_params_file"]))
    print(os.path.join(file_paths["module"], file_paths["veh_params_file"]))


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

import json
from dataclasses import dataclass
from pathlib import Path
import os
import configparser


# own code base
from commonroad_raceline_planner.configuration.base_config import BaseConfigFactory

# typing
from typing import Union


@dataclass
class StepsizeConfig:
    stepsize_preperation: float
    stepsize_regression: float
    stepsize_interp_after_opt: float


@dataclass
class SmoothingConfig:
    k_reg: float
    s_reg: float


@dataclass
class CurvatureCalcConfig:
    d_preview_curv: float
    d_review_curv: float
    d_preview_head: float
    d_review_head: float


@dataclass
class VehicleConfig:
    v_max: float
    length: float
    width: float
    mass: float
    drag_coefficient: float
    curvature_limit: float
    g: float


@dataclass
class VelocityCalcConfig:
    dyn_model_exp: float
    velocity_profile_filter: Union[float, None]


@dataclass
class GeneralConfig:
    ggv_file_path: Union[Path, str]
    ax_max_machines_file_path: [Path, str]
    stepsize_config: StepsizeConfig
    smoothing_config: SmoothingConfig
    curvature_calc_config: CurvatureCalcConfig
    vehicle_config: VehicleConfig
    velocity_calc_config: VelocityCalcConfig


# - class factory
class GeneralConfigFactory(BaseConfigFactory):
    """
    General _execution_config factory
    """

    def generate_from_racecar_ini(
        self,
        path_to_racecar_ini: Union[Path, str],
    ) -> GeneralConfig:
        """
        Generates General Config from racecar ini
        :param path_to_racecar_ini: absolut path to racecar ini
        :return: general _execution_config object
        """
        # load .ini file
        self._load_ini(file_path=path_to_racecar_ini)

        # ggv diagram
        ggv_file_path = json.loads(
            self._parser.get(section="GENERAL_OPTIONS", option="ggv_file")
        )

        # max long acceleration
        ax_max_machines_file_path = json.loads(
            self._parser.get("GENERAL_OPTIONS", "ax_max_machines_file")
        )

        # stepsize opts
        conf = json.loads(
            self._parser.get(section="GENERAL_OPTIONS", option="stepsize_opts")
        )
        stepsize_config = StepsizeConfig(
            stepsize_preperation=conf["stepsize_prep"],
            stepsize_regression=conf["stepsize_reg"],
            stepsize_interp_after_opt=conf["stepsize_interp_after_opt"],
        )

        # smoothing _execution_config
        conf = json.loads(
            self._parser.get(section="GENERAL_OPTIONS", option="reg_smooth_opts")
        )
        smoothing_config = SmoothingConfig(
            k_reg=conf["k_reg"],
            s_reg=conf["s_reg"],
        )

        # curvature calc _execution_config
        conf = json.loads(
            self._parser.get(section="GENERAL_OPTIONS", option="curv_calc_opts")
        )
        curvature_calc_config = CurvatureCalcConfig(
            d_preview_curv=conf["d_preview_curv"],
            d_review_curv=conf["d_review_curv"],
            d_preview_head=conf["d_preview_head"],
            d_review_head=conf["d_review_head"],
        )

        # vehicle params
        conf = json.loads(
            self._parser.get(section="GENERAL_OPTIONS", option="veh_params")
        )
        vehicle_config = VehicleConfig(
            v_max=conf["v_max"],
            length=conf["length"],
            width=conf["width"],
            mass=conf["mass"],
            drag_coefficient=conf["dragcoeff"],
            curvature_limit=conf["curvlim"],
            g=conf["g"],
        )

        # vehicle params
        conf = json.loads(
            self._parser.get(section="GENERAL_OPTIONS", option="vel_calc_opts")
        )
        velocity_calc_config = VelocityCalcConfig(
            dyn_model_exp=conf["dyn_model_exp"],
            velocity_profile_filter=conf["vel_profile_conv_filt_window"],
        )

        return GeneralConfig(
            ggv_file_path=ggv_file_path,
            ax_max_machines_file_path=ax_max_machines_file_path,
            stepsize_config=stepsize_config,
            smoothing_config=smoothing_config,
            curvature_calc_config=curvature_calc_config,
            vehicle_config=vehicle_config,
            velocity_calc_config=velocity_calc_config,
        )


def setup_vehicle_parameters(config):
    file_paths = config.file_paths

    parser = configparser.ConfigParser()
    pars = {}

    parser.read(os.path.join(file_paths["module"], file_paths["veh_params_file"]))
    print(os.path.join(file_paths["module"], file_paths["veh_params_file"]))

    # Add attributes to dict
    add_to_dict(pars, "ggv_file", json.loads(parser.get("GENERAL_OPTIONS", "ggv_file")))
    add_to_dict(
        pars,
        "ax_max_machines_file",
        json.loads(parser.get("GENERAL_OPTIONS", "ax_max_machines_file")),
    )
    add_to_dict(
        pars,
        "stepsize_opts",
        json.loads(parser.get("GENERAL_OPTIONS", "stepsize_opts")),
    )
    add_to_dict(
        pars,
        "reg_smooth_opts",
        json.loads(parser.get("GENERAL_OPTIONS", "reg_smooth_opts")),
    )
    add_to_dict(
        pars, "veh_params", json.loads(parser.get("GENERAL_OPTIONS", "veh_params"))
    )
    add_to_dict(
        pars,
        "vel_calc_opts",
        json.loads(parser.get("GENERAL_OPTIONS", "vel_calc_opts")),
    )

    if config.opt_type == "shortest_path":
        pars["optim_opts"] = json.loads(
            parser.get("OPTIMIZATION_OPTIONS", "optim_opts_shortest_path")
        )
    elif config.opt_type == "mincurv":
        pars["optim_opts"] = json.loads(
            parser.get("OPTIMIZATION_OPTIONS", "optim_opts_mincurv")
        )
    elif config.opt_type == "mintime":
        pars["curv_calc_opts"] = json.loads(
            parser.get("GENERAL_OPTIONS", "curv_calc_opts")
        )
        pars["optim_opts"] = json.loads(
            parser.get("OPTIMIZATION_OPTIONS", "optim_opts_mintime")
        )
        pars["vehicle_params_mintime"] = json.loads(
            parser.get("OPTIMIZATION_OPTIONS", "vehicle_params_mintime")
        )
        pars["tire_params_mintime"] = json.loads(
            parser.get("OPTIMIZATION_OPTIONS", "tire_params_mintime")
        )
        pars["pwr_params_mintime"] = json.loads(
            parser.get("OPTIMIZATION_OPTIONS", "pwr_params_mintime")
        )

        # modification of mintime options/parameters
        pars["optim_opts"]["var_friction"] = config.mintime_opts["var_friction"]
        pars["optim_opts"]["warm_start"] = config.mintime_opts["warm_start"]
        pars["vehicle_params_mintime"]["wheelbase"] = (
            pars["vehicle_params_mintime"]["wheelbase_front"]
            + pars["vehicle_params_mintime"]["wheelbase_rear"]
        )

    if not (
        config.opt_type == "mintime"
        and not config.mintime_opts["recalc_vel_profile_by_tph"]
    ):
        add_to_dict(
            file_paths,
            "ggv_file",
            os.path.join(
                file_paths["module"], "inputs", "veh_dyn_info", pars["ggv_file"]
            ),
        )
        add_to_dict(
            file_paths,
            "ax_max_machines_file",
            os.path.join(
                file_paths["module"],
                "inputs",
                "veh_dyn_info",
                pars["ax_max_machines_file"],
            ),
        )

    return pars


def add_to_dict(dictionary, key, value) -> None:
    """
    add value to dict
    """
    dictionary[key] = value

import numpy as np
from dataclasses import dataclass, field
from pathlib import Path

# typing
from typing import Optional, List, Union

from commonroad_raceline_planner.configuration.base_config import BaseConfigFactory


@dataclass
class DebugConfig: # Debug options
    debug: bool
    mincurv_curv_lin: bool
    raceline: bool
    imported_bounds: bool
    raceline_curv: bool
    racetraj_vel: bool
    racetraj_vel_3d: bool
    racetraj_vel_3d_stepsize: bool
    spline_normals: bool
    mintime_plots: bool


@dataclass
class ImportConfig:
    flip_imp_track: bool
    set_new_start: bool
    new_start: np.ndarray
    num_laps: int
    min_track_width: Optional[float] = None



@dataclass
class MintimeConfig:
    warm_start: bool
    reopt_mintime_solution: bool
    recalc_vel_profile_by_tph: bool
    tpadata: Optional[str] = None
    var_friction: Optional[str] = None



@dataclass  # Lap time calculation table
class LapTimeMatrixConfig:
    use_lap_time_mat: bool
    gg_scale_range: List[float]
    gg_scale_stepsize: float
    top_speed_range: List[float]
    top_speed_stepsize: float
    file: str


@dataclass
class FilePathConfig:
    veh_params_file: str
    track_name: str
    track_file: str
    traj_race_export: str
    lap_time_mat_export: str
    reference_path_export: str
    velocity_profile_export: str
    # Attributes for Friction Optimization
    tpamap: Optional[str, None] = None
    tpadata: Optional[str, None] = None


@dataclass
class ExecutionConfig:
    debug_config: DebugConfig
    import_config: ImportConfig
    mintime_config: MintimeConfig
    lap_time_matrix_config: LapTimeMatrixConfig
    filepath_config: FilePathConfig
    optimization_type: str



class ExecutionConfigFactory(BaseConfigFactory):
    """
    Generates execution config
    """

    def generate_from_yml(
            self,
            path_to_yml: Union[Path, str],
    ) -> ExecutionConfig:
        """
        Generates Optimization Config from racecar ini
        :param path_to_racecar_ini: absolut path to racecar ini
        :param optimization_type: optimization type
        :return: optimization config object
        """

        # load config
        config_dict = self._load_yml(file_path=path_to_yml)

        # debug config
        debug_config = DebugConfig(
            debug=config_dict["debug"]["debug"],
            mincurv_curv_lin=config_dict["debug"]["plotopts"]["mincurv_curv_lin"],
            raceline=config_dict["debug"]["plotopts"]["raceline"],
            imported_bounds=config_dict["debug"]["plotopts"]["imported_bounds"],
            raceline_curv=config_dict["debug"]["plotopts"]["raceline_curv"],
            racetraj_vel=config_dict["debug"]["plotopts"]["racetraj_vel"],
            racetraj_vel_3d=config_dict["debug"]["plotopts"]["racetraj_vel_3d"],
            racetraj_vel_3d_stepsize=config_dict["debug"]["plotopts"]["racetraj_vel_3d_stepsize"],
            spline_normals=config_dict["debug"]["plotopts"]["spline_normals"],
            mintime_plots=config_dict["debug"]["plotopts"]["mintime_plots"],
        )

        # import config
        import_config = ImportConfig(
            flip_imp_track=config_dict["import_opts"]["flip_imp_track"],
            set_new_start=config_dict["import_opts"]["set_new_start"],
            new_start=config_dict["import_opts"]["new_start"],
            min_track_width=config_dict["import_opts"]["min_track_width"],
            num_laps=config_dict["import_opts"]["num_laps"],
        )

        # mintime config
        mintime_config = MintimeConfig(
            tpadata=config_dict["mintime_opts"]["tpadata"],
            warm_start=config_dict["mintime_opts"]["warm_start"],
            var_friction=config_dict["mintime_opts"]["var_friction"],
            reopt_mintime_solution=config_dict["mintime_opts"]["reopt_mintime_solution"],
            recalc_vel_profile_by_tph=config_dict["mintime_opts"]["recalc_vel_profile_by_tph"],
        )

        # lap time matrix config
        lap_time_matrix_config = LapTimeMatrixConfig(
            use_lap_time_mat=config_dict["lap_time_mat_opts"]["use_lap_time_mat"],
            gg_scale_range=config_dict["lap_time_mat_opts"]["gg_scale_range"],
            gg_scale_stepsize=config_dict["lap_time_mat_opts"]["gg_scale_stepsize"],
            top_speed_range=config_dict["lap_time_mat_opts"]["top_speed_range"],
            top_speed_stepsize=config_dict["lap_time_mat_opts"]["top_speed_stepsize"],
            file=config_dict["lap_time_mat_opts"]["file"],
        )

        # file path opts
        filepath_config = FilePathConfig(
            veh_params_file=config_dict["file_paths"]["veh_params_file"],
            track_name=config_dict["file_paths"]["track_name"],
            track_file=config_dict["file_paths"]["track_file"],
            traj_race_export=config_dict["file_paths"]["traj_race_export"],
            lap_time_mat_export=config_dict["file_paths"]["lap_time_mat_export"],
            reference_path_export=config_dict["file_paths"]["reference_path_export"],
            velocity_profile_export=config_dict["file_paths"]["velocity_profile_export"],
        )

        # optimization type
        optimization_type = config_dict["opt_type"]

        return ExecutionConfig(
            debug_config=debug_config,
            import_config=import_config,
            mintime_config=mintime_config,
            lap_time_matrix_config=lap_time_matrix_config,
            filepath_config=filepath_config,
            optimization_type=optimization_type
        )













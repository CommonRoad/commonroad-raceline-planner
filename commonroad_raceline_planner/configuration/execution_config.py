import numpy as np
from dataclasses import dataclass, field
import yaml

# typing
from typing import Optional, Dict, List


@dataclass
class DebugConfiguration: # Debug options
    debug: bool = True
    plot_opts: Dict[str, bool] = field(default_factory=lambda: {
        "mincurv_curv_lin": False,
        "raceline": True,
        "imported_bounds": False,
        "raceline_curv": True,
        "racetraj_vel": True,
        "racetraj_vel_3d": False,
        "racetraj_vel_3d_stepsize": 1.0,
        "spline_normals": False,
        "mintime_plots": False
    })

    def __getitem__(self, key):
        return getattr(self, key)


@dataclass # Import options
class ImportOptions:
    flip_imp_track: bool = False
    set_new_start: bool = False
    new_start: np.ndarray = np.array([0.0, -47.0])
    min_track_width: Optional[float] = None
    num_laps: int = 1


@dataclass # minimum time optimization options
class MintimeOptions:
    tpadata: Optional[str] = None
    warm_start: bool = False
    var_friction: Optional[str] = None
    reopt_mintime_solution: bool = False
    recalc_vel_profile_by_tph: bool = False


@dataclass  # Lap time calculation table
class LapTimeMatrixOptions:
    use_lap_time_mat: bool = False
    gg_scale_range: List[float] = field(default_factory=lambda: [0.3, 1.0])
    gg_scale_stepsize: float = 0.05
    top_speed_range: List[float] = field(default_factory=lambda: [100.0, 150.0])
    top_speed_stepsize: float = 5.0
    file: str = "lap_time_matrix.csv"

    def __getitem__(self, key):
        return getattr(self, key)


@dataclass
class FilePathConfiguration:
    veh_params_file: str = "racecar.ini"
    track_name: str = "lanelet_data"
    track_file: str = "lanelet_data.csv"
    traj_race_export: str = "outputs/traj_race_cl.csv"
    lap_time_mat_export: str = "outputs/lap_time_matrix.csv"
    reference_path_export: str = "outputs/reference_path.json"
    velocity_profile_export: str = 'outputs/reference_path.json'
    # Attributes for Friction Optimization
    tpamap: str = field(init=False, default=None)
    tpadata: str = field(init=False, default=None)

    def __getitem__(self, key):
        return getattr(self, key)


@dataclass
class ExecutionConfig:
    debug: DebugConfiguration = DebugConfiguration()
    import_opts: ImportOptions = ImportOptions()
    mintime_opts: MintimeOptions = MintimeOptions()
    lap_time_mat_opts: LapTimeMatrixOptions = LapTimeMatrixOptions()
    file_paths: FilePathConfiguration = FilePathConfiguration()
    opt_type: str = "shortest_path"

    @staticmethod
    def load(file_path: str) -> 'ExecutionConfig':
        with open(file_path, 'r') as file:
            config_dict = yaml.safe_load(file)
        return ExecutionConfig(**config_dict)

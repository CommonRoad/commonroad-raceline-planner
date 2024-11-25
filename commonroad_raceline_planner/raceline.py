from dataclasses import dataclass
import warnings
from pathlib import Path

import numpy as np

# own code
from commonroad_raceline_planner.util.io import export_traj_race

# typing
from typing import Union


@dataclass
class RaceLine:
    """
    Raceline
    """
    points: np.ndarray
    length_per_point: np.ndarray
    velocity_long_per_point: np.ndarray
    acceleration_long_per_point: np.ndarray
    curvature_per_point: np.ndarray
    heading_per_point: np.ndarray
    num_points: Union[int, None] = None
    sanity: Union[bool, None] = None
    closed: bool = False

    def __post_init__(self):
        self.num_points = self.points.shape[0]
        self.sanity = self.sanity_check()

    def to_7d_np_array(self) -> np.ndarray:
        """
        Convert raceline to np.ndarray
        :return: (num_point, 7) ndarray.
        np.ndarray(
        length_per_point, points, heading_per_point, curvature_per_point, velocity_long_per_point, acceleration_long_per_point
        )
        """
        return np.column_stack(
            (
               self.length_per_point,
               self.points,
               self.heading_per_point,
               self.curvature_per_point,
               self.velocity_long_per_point,
               self.acceleration_long_per_point
            )
        )

    def sanity_check(self) -> bool:
        """
        Sanity check fo racline
        :return: returns false if certain parameters are wrong
        """
        sanity: bool = True
        if(
            not
            self.points.shape[0]
            == self.length_per_point.shape[0]
            == self.velocity_long_per_point.shape[0]
            == self.acceleration_long_per_point.shape[0]
            == self.curvature_per_point.shape[0]
            == self.heading_per_point.shape[0]
            == self.num_points
        ):
            warnings.warn(
                f"raceline has mismatching length of data: \n "
                f"points={self.points.shape[0]}  \n"
                f"num_length_per_point={self.length_per_point.shape[0]}  \n"
                f"num_acceleration_long_per_point={self.acceleration_long_per_point.shape[0]}  \n"
                f"num_curvature_per_point={self.curvature_per_point.shape[0]}  \n"
                f"num_heading_per_point={self.heading_per_point.shape[0]}  \n"
                f"num_points={self.num_points}  \n"
            )
            sanity = False

        return sanity

    def close_raceline(self) -> None:
        """
        Close raceline
        """
        self.points = np.hstack((self.points, self.points[0]))
        self.length_per_point = np.hstack((self.length_per_point, self.length_per_point[0]))
        self.velocity_long_per_point = np.hstack((self.velocity_long_per_point, self.velocity_long_per_point[0]))
        self.acceleration_long_per_point = np.hstack((self.acceleration_long_per_point, self.acceleration_long_per_point[0]))
        self.curvature_per_point = np.hstack((self.curvature_per_point, self.curvature_per_point[0]))
        self.heading_per_point = np.hstack((self.curvature_per_point, self.curvature_per_point[0]))
        self.num_points = self.points.shape[0]
        self.sanity = self.sanity_check()

    def export_trajectory_to_csv_file(
            self,
            export_path: Union[Path, str],
            ggv_file_path: Union[Path, str]
    ) -> None:
        """
        Export trajectory to csv file.
        :param export_path:
        :param ggv_file_path:
        """
        export_traj_race(
            traj_race_export=export_path,
            ggv_file=ggv_file_path,
            traj_race=self.to_7d_np_array()
        )
        print(f'Exported trajectory to {export_path}')


class RaceLineFactory:
    """
    Generates raceline
    """

    @staticmethod
    def generate_raceline(
        points: np.ndarray,
        length_per_point: np.ndarray,
        velocity_long_per_point: np.ndarray,
        acceleration_long_per_point: np.ndarray,
        curvature_per_point: np.ndarray,
        heading_per_point: np.ndarray,
        closed: bool
    ) -> RaceLine:
        """
        Generates race line
        :param points:
        :param length_per_point:
        :param velocity_long_per_point:
        :param acceleration_long_per_point:
        :param curvature_per_point:
        :param heading_per_point:
        :return:
        """
        return RaceLine(
            points=points,
            length_per_point=length_per_point,
            velocity_long_per_point=velocity_long_per_point,
            acceleration_long_per_point=acceleration_long_per_point,
            curvature_per_point=curvature_per_point,
            heading_per_point=heading_per_point,
            closed=closed
        )


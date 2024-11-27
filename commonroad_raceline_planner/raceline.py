from dataclasses import dataclass
import warnings
from pathlib import Path

import numpy as np
from scipy.spatial.kdtree import KDTree

# own code
from commonroad_raceline_planner.utils.io import export_traj_race

# typing
from typing import Union, Tuple


@dataclass
class RaceLine:
    """
    A cr raceline object containing information on the generated raceline.

    :param points: (n,2) x,y points
    :param length_per_point: (n,) arclength (accumulated) per point
    :param velocity_long_per_point: (n,) velocity per point
    :param acceleration_long_per_point: (n,) acceleration per point
    :param curvature_per_point: (n,) curvature per point
    :param num_points: number of points
    :param closed: true, if racetrack is closed
    :param sanity: true, if dimensions make sense
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
                self.acceleration_long_per_point,
            )
        )

    def get_closest_idx(self, point: np.ndarray) -> int:
        """
        Get idx of closest point on raceline
        :param point: (2,) numpy array
        :return: index
        """
        kd_tree: KDTree = KDTree(self.points)
        _, idx = kd_tree.query(point)
        return idx

    def get_closest_point(self, point: np.ndarray) -> Tuple[int, np.ndarray]:
        """
        Get idx and coords of closest point on raceline
        :param point: closest point on raceline
        :return: Tuple (idx, coordinates)
        """
        idx: int = self.get_closest_idx(point)
        return (idx, self.points[idx])

    def get_velocity_at_position_with_lookahead(
        self, position: np.ndarray, lookahead_s: float = 2.0
    ) -> float:
        """
        Get velocity at position. Uses closest point of raceline
        :param position: (2,) position array
        :return: velocity
        """
        idx_0: int = self.get_closest_idx(position)
        v_0: float = self.velocity_long_per_point[idx_0]
        s_0: float = self.length_per_point[idx_0]

        for idx in range(self.velocity_long_per_point[idx_0:].shape[0]):
            idx_1 = idx_0 + idx
            v_1: float = self.velocity_long_per_point[idx_1]
            s_1: np.ndarray = self.length_per_point[idx_1]

            delta_s = s_1 - s_0
            delta_v = v_0 + (v_1 - v_0) / 2

            if delta_v == 0:
                delta_t: float = delta_s / v_0
            else:
                delta_t: float = (s_1 - s_0) / (v_0 + (v_1 - v_0) / 2)

            if delta_t >= lookahead_s:
                break

        return self.velocity_long_per_point[idx_1]

    def sanity_check(self) -> bool:
        """
        Sanity check fo racline
        :return: returns false if certain parameters are wrong
        """
        sanity: bool = True
        if (
            not self.points.shape[0]
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
        Closes raceline by adding the first point to the end
        """
        self.points = np.hstack((self.points, self.points[0]))
        self.length_per_point = np.hstack(
            (self.length_per_point, self.length_per_point[0])
        )
        self.velocity_long_per_point = np.hstack(
            (self.velocity_long_per_point, self.velocity_long_per_point[0])
        )
        self.acceleration_long_per_point = np.hstack(
            (self.acceleration_long_per_point, self.acceleration_long_per_point[0])
        )
        self.curvature_per_point = np.hstack(
            (self.curvature_per_point, self.curvature_per_point[0])
        )
        self.heading_per_point = np.hstack(
            (self.curvature_per_point, self.curvature_per_point[0])
        )
        self.num_points = self.points.shape[0]
        self.sanity = self.sanity_check()

    def export_trajectory_to_csv_file(
        self, export_path: Union[Path, str], ggv_file_path: Union[Path, str]
    ) -> None:
        """
        Export trajectory to csv file.
        :param export_path: path to which the trajectory should be safed as csv
        :param ggv_file_path: ggv file path
        """
        export_traj_race(
            traj_race_export=export_path,
            ggv_file=ggv_file_path,
            traj_race=self.to_7d_np_array(),
        )
        print(f"Exported trajectory to {export_path}")


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
        closed: bool,
    ) -> RaceLine:
        """
        Generates race line
        :param points: (n,2) x,y points
        :param length_per_point: arc length per point (accumulated)
        :param velocity_long_per_point: velocity per point
        :param acceleration_long_per_point: acceleration per point
        :param curvature_per_point: curvature per point
        :param heading_per_point: heading per point
        :return: cr raceline
        """
        return RaceLine(
            points=points,
            length_per_point=length_per_point,
            velocity_long_per_point=velocity_long_per_point,
            acceleration_long_per_point=acceleration_long_per_point,
            curvature_per_point=curvature_per_point,
            heading_per_point=heading_per_point,
            closed=closed,
        )

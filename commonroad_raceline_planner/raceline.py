from dataclasses import dataclass
import warnings

import numpy as np

# typing
from typing import Union

from shapely.creation import points

from commonroad_raceline_planner.ractetrack import DtoFTM


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

    def sanity_check(self) -> bool:
        """
        Sanity check fo racline
        :return: returns false if certain parameters are wrong
        """
        sanity: bool = True
        if(
            not
            self.points.shape[0]
            == self.length_per_point[0]
            == self.velocity_long_per_point.shape[0]
            == self.acceleration_long_per_point.shape[0] == self.curvature_per_point.shape[0]
            == self.heading_per_point.shape[0]
            == self.num_points
        ):
            warnings.warn(
                f"raceline has mismatching length of data"
            )
            sanity = False

        return sanity

    def close_raceline(self) -> None:
        """
        Close raceline
        """
        # TODO: implement
        self.points = np.hstack((self.points, self.points[0]))
        self.length_per_point = np.hstack((self.length_per_point, self.length_per_point[0]))
        self.velocity_long_per_point = np.hstack((self.velocity_long_per_point, self.velocity_long_per_point[0]))
        self.acceleration_long_per_point = np.hstack((self.acceleration_long_per_point, self.acceleration_long_per_point[0]))
        self.curvature_per_point = np.hstack((self.curvature_per_point, self.curvature_per_point[0]))
        self.heading_per_point = np.hstack((self.curvature_per_point, self.curvature_per_point[0]))
        self.num_points = self.points.shape[0]
        self.sanity = self.sanity_check()


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


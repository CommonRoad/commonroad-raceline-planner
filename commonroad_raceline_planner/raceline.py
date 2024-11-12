from dataclasses import dataclass
import warnings

import numpy as np

# typing
from typing import Union


@dataclass
class DtoRacing:
    """
    Data transfer object resembling an actual raceline.
    """
    points: Union[np.ndarray, None] = None
    length_per_point: Union[np.ndarray, None] = None
    velocity_long_per_point: Union[np.ndarray, None] = None
    acceleration_long_per_point: Union[np.ndarray, None] = None
    curvature_per_point: Union[np.ndarray, None] = None
    heading_per_point: Union[np.ndarray, None] = None
    num_points: Union[int, None] = None
    sanity: Union[bool, None] = None


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


class RaceLineFactory:
    """
    Generates raceline
    """

    @staticmethod
    def generate_race_line(
        points: np.ndarray,
        length_per_point: np.ndarray,
        velocity_long_per_point: np.ndarray,
        acceleration_long_per_point: np.ndarray,
        curvature_per_point: np.ndarray,
        heading_per_point: np.ndarray,
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
            heading_per_point=heading_per_point
        )








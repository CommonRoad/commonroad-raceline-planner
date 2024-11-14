import copy
from dataclasses import dataclass
import warnings
import numpy as np
import math
from yaml import warnings

# commonroad
from commonroad_raceline_planner.util.exceptions import (
    TrackDataInvalidException,
    TrackNotClosedException
)



# typing
from typing import Union, Optional




@dataclass
class RaceTrack:
    """
    Race track from centerline with left and right bounds
    """
    x_m: np.ndarray
    y_m: np.ndarray
    w_tr_right_m: np.ndarray
    w_tr_left_m: np.ndarray
    interpoint_length: Union[np.ndarray, None] = None
    track_length_per_point: Union[np.ndarray, None] = None
    track_length: Union[float, None] = None
    num_points: Union[int, None] = None

    def __post_init__(self):
        self.num_points = self.x_m.shape[0]
        self.interpoint_length = np.linalg.norm(np.hstack(self.x_m, self.y_m), ord=2, axis=1)
        self.track_length_per_point = np.cumsum(self.interpoint_length)
        self.track_length_per_point = np.insert(self.track_length_per_point, 0, 0.0)
        self.track_length = self.track_length_per_point[-1]
        self.sanity_check()

    def sanity_check(
            self,
    ) -> None:
        """
        Sanity check for racetrack data class
        """

        # check same dimensions
        if(
                not self.x_m.shape[0] == self.y_m.shape[0] == self.w_tr_left_m.shape[0] ==
                    self.w_tr_right_m.shape[0] == self.num_points
        ):
            raise TrackDataInvalidException(
                f"The computed race track data have not the same number of points: "
                f"x_m={self.x_m.shape[0]}, y_m={self.y_m.shape[0]}, w_tr_left_m={self.w_tr_left_m.shape[0]}, w_tr_right_m={self.w_tr_right_m.shape[0]}"
            )


class DtoRacetrack:
    def __init__(
            self,
            race_track: RaceTrack
    ) -> None:
        """
        Data Transfer Object (flow object) of a racetrack
        :param race_track:
        """
        self._original_track: RaceTrack = race_track

        # copy race track values so we can manipulate them
        self.x_m: np.ndarray = copy.copy(race_track.x_m)
        self.y_m: np.ndarray = copy.copy(race_track.y_m)
        self.w_tr_right_m: np.ndarray = copy.copy(race_track.w_tr_right_m)
        self.w_tr_left_m: np.ndarray = copy.copy(race_track.w_tr_left_m)
        self.interpoint_length: np.ndarray = copy.copy(race_track.interpoint_length)
        self.track_length_per_point: np.ndarray = copy.copy(race_track.track_length_per_point)
        self.track_length: float = copy.copy(race_track.track_length_per_point)
        self.num_points: int = copy.copy(race_track.num_points)
        self.is_closed: bool = False
        self.is_interpolated: bool = False

    def close_racetrack(
            self,
            forced_recalc: bool = False
    ) -> None:
        """
        Close racetrack by appending first point to end again
        """
        if self.is_closed and not forced_recalc:
            warnings.warn("Racetrack already closed, wont close it twice")
        else:
            self.x_m = np.hstack(self.x_m, self.x_m[0])
            self.y_m = np.hstack(self.y_m, self.y_m[0])
            self.w_tr_right_m = np.hstack(self.w_tr_right_m, self.w_tr_right_m[0])
            self.w_tr_left_m = np.hstack(self.w_tr_left_m, self.w_tr_left_m[0])
            self.num_points = self.x_m.shape[0]
            self.interpoint_length = np.linalg.norm(np.hstack(self.x_m, self.y_m), ord=2, axis=1)
            self.track_length_per_point = np.cumsum(self.interpoint_length)
            if (self.track_length_per_point[0] != 0.0):
                self.track_length_per_point = np.insert(self.track_length_per_point, 0, 0.0)
            self.track_length = self.track_length_per_point[-1]
            self.is_closed = True

    def open_racetrack(self) -> None:
        """
        Reopens race track
        """
        if not self.is_closed:
            warnings.warn("race track is not closed, cannot open it")
        else:
            self.x_m = self.x_m[:-1]
            self.y_m = self.y_m[:-1]
            self.w_tr_right_m = self.w_tr_right_m[:-1]
            self.w_tr_left_m = self.w_tr_left_m[:-1]
            self.num_points = self.num_points - 1
            self.interpoint_length = self.interpoint_length[:-1]
            self.track_length_per_point = self.track_length_per_point[:-1]
            self.track_length = self.track_length_per_point[-1]
            self.is_closed = False


    def linear_interpol_racetrack(
            self,
            interpol_stepsize: float
    ) -> None:
        """
        Lineraly interpolate race track. The race track must be closed.
        :param interpol_stepsize: interpolation stepsize
        """
        if not self.is_closed:
            warnings.warn("starting interpolation but race track is not closed")
        else:
            pass

        if self.is_interpolated:
            warnings.warn("racetrack is already interpolated wont interpolate again")
        else:
            # calculate desired lenghts depending on specified stepsize (+1 because last element is included)
            num_points: int = math.ceil(self.track_length / interpol_stepsize) + 1
            interpoint_interpol_dist = np.linspace(
                start=0.0,
                stop=self.track_length,
                num=num_points
            )

            self.x_m = np.interp(interpoint_interpol_dist, self.track_length, self.x_m)
            self.y_m = np.interp(interpoint_interpol_dist, self.track_length, self.y_m)
            self.w_tr_left_m = np.interp(interpoint_interpol_dist, self.track_length, self.w_tr_left_m)
            self.w_tr_right_m = np.interp(interpoint_interpol_dist, self.track_length, self.w_tr_right_m)
            self.num_points = self.x_m.shape[0]
            self.interpoint_length = np.linalg.norm(np.hstack(self.x_m, self.y_m), ord=2, axis=1)
            self.track_length_per_point = np.cumsum(self.interpoint_length)
            if (self.track_length_per_point[0] != 0.0):
                self.track_length_per_point = np.insert(self.track_length_per_point, 0, 0.0)
            self.track_length_per_point = np.insert(self.track_length_per_point, 0, 0.0)
            self.track_length = self.track_length_per_point[-1]

            self.is_interpolated = True
            # interpolating also reopens race track, so close it again
            self.is_closed = False
            self.close_racetrack()

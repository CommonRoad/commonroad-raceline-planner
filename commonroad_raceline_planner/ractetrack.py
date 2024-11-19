import copy
from dataclasses import dataclass
import numpy as np
import warnings

from commonroad.scenario.scenario import Scenario

# commonroad
from commonroad_raceline_planner.util.exceptions import (
    TrackDataInvalidException,
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
    cr_scenario: Union[Scenario, None] = None

    def __post_init__(self):
        self.num_points = self.x_m.shape[0]
        self.interpoint_length = calc_interpoint_length(
            x_m=self.x_m,
            y_m=self.y_m
        )
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


    def to_4d_np_array(self) -> np.ndarray:
        """
        Convert to 4d numpy array
        :return: np.ndarray[x_m, y_m, w_tr_right_m, w_tr_left_m] -> dim = (num_points, 4)
        """
        return np.vstack(
            (self.x_m, self.y_m, self.w_tr_right_m, self.w_tr_left_m)
        )

    def to_2d_np_array(self) -> np.ndarray:
        """
        Convert to 2d numpy array
        :return: np.ndarray[x_m,y_m] -> dim = (num_points, 2)
        """
        return np.vstack(
            (self.x_m, self.y_m)
        )




class DtoRacetrack:
    def __init__(
            self,
            race_track: RaceTrack,
            x_m: np.ndarray,
            y_m: np.ndarray,
            w_tr_right_m: np.ndarray,
            w_tr_left_m: np.ndarray,
            interpoint_length: np.ndarray,
            track_length_per_point: np.ndarray,
            track_length: float,
            num_points: int,
            is_closed: bool = False,
            is_interpolated: bool = False,
            is_spline_approximated: bool = False,
            perform_sanity_check: bool = False
    ) -> None:
        """
        Data Transfer Object (flow object) of a racetrack
        :param race_track:
        """
        self.original_track: RaceTrack = race_track

        # copy race track values so we can manipulate them
        self.x_m: np.ndarray = x_m
        self.y_m: np.ndarray = y_m
        self.w_tr_right_m: np.ndarray = w_tr_right_m
        self.w_tr_left_m: np.ndarray = w_tr_left_m
        self.interpoint_length: np.ndarray = interpoint_length
        self.track_length_per_point: np.ndarray = track_length_per_point
        self.track_length: float = track_length
        self.num_points: int = num_points
        self.is_closed: bool = is_closed
        self.is_interpolated: bool = is_interpolated
        self.is_spline_approximated: bool = is_spline_approximated

        if perform_sanity_check:
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
                f"x_m={self.x_m.shape[0]}, y_m={self.y_m.shape[0]}, "
                f"w_tr_left_m={self.w_tr_left_m.shape[0]}, w_tr_right_m={self.w_tr_right_m.shape[0]}"
            )

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
            # add first point to end and 0.0 at first race-track length
            self.x_m = np.hstack((self.x_m, self.x_m[0]))
            self.y_m = np.hstack((self.y_m, self.y_m[0]))
            self.w_tr_right_m = np.hstack((self.w_tr_right_m, self.w_tr_right_m[0]))
            self.w_tr_left_m = np.hstack((self.w_tr_left_m, self.w_tr_left_m[0]))
            self.num_points = self.x_m.shape[0]
            self.interpoint_length = calc_interpoint_length(
                x_m=self.x_m,
                y_m=self.y_m
            )
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

    def to_4d_np_array(self) -> np.ndarray:
        """
        Convert to 4d numpy array
        :return: np.ndarray[x_m, y_m, w_tr_right_m, w_tr_left_m] -> dim = (num_points, 4)
        """
        return np.swapaxes(
            np.vstack(
                (self.x_m, self.y_m, self.w_tr_right_m, self.w_tr_left_m)
            ),
            axis1=1,
            axis2=0
        )

    def to_2d_np_array(self) -> np.ndarray:
        """
        Convert to 2d numpy array
        :return: np.ndarray[x_m,y_m] -> dim = (num_points, 2)
        """
        return np.swapaxes(
                    np.vstack(
                        (self.x_m, self.y_m)
                    ),
                    axis1=1,
                    axis2=0
                )




class DtoRacetrackFactory:

    def generate_from_racetrack(
            self,
            race_track: RaceTrack,
            is_closed: bool = False,
            is_interpolated: bool = False,
            is_spline_approximated: bool = False
    ) -> DtoRacetrack:
        """
        Generates DtoRacetrack instance from RaceTrack instance
        :param race_track:
        :param is_closed:
        :param is_interpolated:
        :param is_spline_approximated:
        :return:
        """
        return DtoRacetrack(
            race_track=race_track,
            x_m=copy.deepcopy(race_track.x_m),
            y_m=copy.deepcopy(race_track.y_m),
            w_tr_right_m=copy.deepcopy(race_track.w_tr_right_m),
            w_tr_left_m=copy.deepcopy(race_track.w_tr_left_m),
            interpoint_length=copy.deepcopy(race_track.interpoint_length),
            track_length_per_point=copy.deepcopy(race_track.track_length_per_point),
            track_length=copy.deepcopy(race_track.track_length),
            num_points=copy.deepcopy(race_track.num_points),
            is_closed=is_closed,
            is_interpolated=is_interpolated,
            is_spline_approximated=is_spline_approximated,
            perform_sanity_check=True
        )

    def generate_from_constructor_values(
            self,
            race_track: RaceTrack,
            x_m: np.ndarray,
            y_m: np.ndarray,
            w_tr_right_m: np.ndarray,
            w_tr_left_m: np.ndarray,
            interpoint_length: np.ndarray,
            track_length_per_point: np.ndarray,
            track_length: float,
            num_points: int,
            is_closed: bool,
            is_interpolated: bool,
            is_spline_approximated: bool,
    ) -> DtoRacetrack:
        """
        Generates DtoRacetrack instance from constructor values
        """
        return DtoRacetrack(
            race_track=race_track,
            x_m=x_m,
            y_m=y_m,
            w_tr_right_m=w_tr_right_m,
            w_tr_left_m=w_tr_left_m,
            interpoint_length=interpoint_length,
            track_length_per_point=track_length_per_point,
            track_length=track_length,
            num_points=num_points,
            is_closed=is_closed,
            is_interpolated=is_interpolated,
            is_spline_approximated=is_spline_approximated,
            perform_sanity_check=True
        )


    def generate_from_centerline_and_bounds(
            self,
            race_track: RaceTrack,
            x_m: np.ndarray,
            y_m: np.ndarray,
            w_tr_right_m: np.ndarray,
            w_tr_left_m: np.ndarray,
            is_closed: bool,
            is_interpolated: bool,
            is_spline_approximated: bool,
    ) -> DtoRacetrack:
        """
        Generate dtoracetrack instance from centerline and bounds
        :param race_track:
        :param x_m:
        :param y_m:
        :param w_tr_right_m:
        :param w_tr_left_m:
        :param is_closed:
        :param is_interpolated:
        :param is_spline_approximated:
        :return:
        """
        num_points = x_m.shape[0]
        interpoint_length = calc_interpoint_length(
            x_m=x_m,
            y_m=y_m
        )
        track_length_per_point = np.cumsum(interpoint_length)
        track_length_per_point = np.insert(track_length_per_point, 0, 0.0)
        track_length = track_length_per_point[-1]

        return DtoRacetrack(
            race_track=race_track,
            x_m=x_m,
            y_m=y_m,
            w_tr_right_m=w_tr_right_m,
            w_tr_left_m=w_tr_left_m,
            interpoint_length=interpoint_length,
            track_length_per_point=track_length_per_point,
            track_length=track_length,
            num_points=num_points,
            is_closed=is_closed,
            is_interpolated=is_interpolated,
            is_spline_approximated=is_spline_approximated,
            perform_sanity_check=True
        )


def calc_interpoint_length(
        x_m: np.ndarray,
        y_m: np.ndarray
) -> np.ndarray:
    """
    Calc interpoint length
    :param x_m: x coordinate
    :param y_m: y coordinate
    :return: array of interpoint lengths
    """
    differences = np.diff(np.vstack((x_m, y_m)), axis=1)
    squared_differences = np.power(differences, 2)
    summed_squared_differences = np.sum(squared_differences, axis=0)
    root_ssd = np.sqrt(summed_squared_differences)
    return root_ssd


if __name__ == "__main__":

    x_m = np.asarray([2.0974, -9.6235, -19.2631])
    y_m = np.asarray([-4.01, 20.41, 40.63])
    d = np.diff(np.vstack((x_m, y_m)), axis=1)
    print(f"diff={d}")
    p2 = np.power(d, 2)
    print(f"power={p2}")
    s = np.sum(p2, axis=0)
    print(f"sum={s}")
    sq = np.sqrt(s)
    print(f"sq={sq}")
    track_length = np.cumsum(sq)
    print(track_length)


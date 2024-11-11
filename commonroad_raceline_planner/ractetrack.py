from dataclasses import dataclass

import numpy as np

# commonroad
from commonroad_raceline_planner.util.exceptions import (
    TrackDataInvalidException,
    TrackNotClosedException
)



# typing
from typing import Union

@dataclass
class RaceTrack:
    x_m: np.ndarray
    y_m: np.ndarray
    w_tr_right_m: np.ndarray
    w_tr_left_m: np.ndarray
    num_points:Union[int, None] = None

    def __post_init__(self):
        self.num_points = self.x_m.shape[0]
        self.sanity_check()





    def sanity_check(
            self,
            interpoint_threshold: float=0.5,
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



        # Check if the filtered points form a closed map -> TODO: move to sanity check of racetrack
        if np.linalg.norm(np.array(self.x_m[0], self.y_m[0]) - np.array(
                [self.x_m[-1], self.y_m[-1]])) > interpoint_threshold:
            raise TrackNotClosedException(f"The map is not closed considering the distance {interpoint_threshold}")


"""
[x_m, y_m, w_tr_right_m, w_tr_left_m]"""
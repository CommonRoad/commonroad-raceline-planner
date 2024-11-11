from dataclasses import dataclass
from typing import Union

import numpy as np

# commonroad
from commonroad_raceline_planner.util.exceptions import TrackDataInvalidException


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





    def sanity_check(self) -> None:
        """
        Sanity check for racetrack data class
        """

        if(not self.x_m.shape[0] == self.y_m.shape[0] == self.w_tr_left_m.shape[0] == self.w_tr_right_m.shape[0] == self.num_points):
            raise TrackDataInvalidException(
                f"The computed race track data have not the same number of points: "
                f"x_m={self.x_m.shape[0]}, y_m={self.y_m.shape[0]}, w_tr_left_m={self.w_tr_left_m.shape[0]}, w_tr_right_m={self.w_tr_right_m.shape[0]}"
            )


        for idx in range(self.num_points):

            # Check if the filtered points form a closed map -> TODO: move to sanity check of racetrack
            if np.linalg.norm(np.array(self.x_m[idx], self.y_m[idx]) - np.array(
                    [filtered_points[-1]['x_m'], filtered_points[-1]['y_m']])) > removing_distance:
                print("The map is not closed.")
            else:
                print("The map is closed.")


"""
[x_m, y_m, w_tr_right_m, w_tr_left_m]"""
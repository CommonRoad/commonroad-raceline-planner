from dataclasses import dataclass
import numpy as np
import warnings

# commonroad
from commonroad.scenario.lanelet import LaneletNetwork, Lanelet

# own code base
from commonroad_raceline_planner.util.path_utils import calc_interpoint_length
from commonroad_raceline_planner.util.exceptions import (
    TrackDataInvalidException,
)

# typing
from typing import Union, Optional, List


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
    lanelet_network: Union[LaneletNetwork, None] = None

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


class RaceTrackFactory:
    """
    Generates CR RaceTrack instance either from csv or CR scenario
    """

    @staticmethod
    def generate_racetrack_from_csv(
        file_path: str,
        vehicle_width: float,
        num_laps: int = 1,
        flip_track: bool = False,
        set_new_start: bool = False,
        new_start: Optional[np.ndarray] = None,
        vehicle_safe_margin_m: float = 0.5
    ) -> RaceTrack:
        """
        Import racetrack from csv

        Inputs:
        file_path:      file path of track.csv containing [x_m,y_m,w_tr_right_m,w_tr_left_m]
        imp_opts:       import options showing if a new starting point should be set or if the direction should be reversed
        width_veh:      vehicle width required to check against track width

        Outputs:
        race_track:   imported track [x_m, y_m, w_tr_right_m, w_tr_left_m]
        """

        # load data from csv file
        csv_data_temp = np.loadtxt(file_path, comments='#', delimiter=',')

        # get coords and track widths out of array
        if np.shape(csv_data_temp)[1] == 3:
            refline_ = csv_data_temp[:, 0:2]
            w_tr_r = csv_data_temp[:, 2] / 2
            w_tr_l = w_tr_r

        elif np.shape(csv_data_temp)[1] == 4:
            refline_ = csv_data_temp[:, 0:2]
            w_tr_r = csv_data_temp[:, 2]
            w_tr_l = csv_data_temp[:, 3]

        elif np.shape(csv_data_temp)[1] == 5:  # omit z coordinate in this case
            refline_ = csv_data_temp[:, 0:2]
            w_tr_r = csv_data_temp[:, 3]
            w_tr_l = csv_data_temp[:, 4]

        else:
            raise IOError("Track file cannot be read!")

        refline_ = np.tile(refline_, (num_laps, 1))
        w_tr_r = np.tile(w_tr_r, num_laps)
        w_tr_l = np.tile(w_tr_l, num_laps)

        # assemble to a single array
        reftrack_imp = np.column_stack((refline_, w_tr_r, w_tr_l))

        # check if imported centerline should be flipped, i.e. reverse direction
        if flip_track:
            reftrack_imp = np.flipud(reftrack_imp)

        # check if imported centerline should be reordered for a new starting point
        if set_new_start:
            ind_start = np.argmin(np.power(reftrack_imp[:, 0] - new_start[0], 2)
                                  + np.power(reftrack_imp[:, 1] - new_start[1], 2))
            reftrack_imp = np.roll(reftrack_imp, reftrack_imp.shape[0] - ind_start, axis=0)

        # check minimum track width for vehicle width plus a small safety margin
        w_tr_min = np.amin(reftrack_imp[:, 2] + reftrack_imp[:, 3])

        if w_tr_min < vehicle_width + vehicle_safe_margin_m:
            warnings.warn(
                f"Minimum track width={np.amin(w_tr_min)}, "
                f"which is close to or smaller than vehicle width {vehicle_width} !"
            )

        # return reftrack_imp

        return RaceTrack(
            x_m=reftrack_imp[:, 0],
            y_m=reftrack_imp[:, 1],
            w_tr_right_m=reftrack_imp[:, 2],
            w_tr_left_m=reftrack_imp[:, 3]
        )



    @staticmethod
    def generate_racetrack_from_cr_scenario(
            lanelet_network: LaneletNetwork,
            vehicle_width: float,
            num_laps: int = 1,
            flip_track: bool = False,
            set_new_start: bool = False,
            new_start: Optional[np.ndarray] = None,
            removing_distance: float = 0.5,
            vehicle_safe_margin_m: float = 0.5
    ) -> RaceTrack:
        """
            Load racetrack from cr scenario

            The CSV file contains the x and y coordinates of each point in the centerline of each lanelet in the lanelet network,
            as well as the distances from each center point to the nearest point in the left and right lines of the lanelet.
            (width_track_left and width_track_right)
            """
        # Open the XML file and read the scenario
        points = []
        lanelets = RaceTrackFactory.sort_lanelets_by_id(lanelet_network)

        for lanelet in lanelets:
            for center_point in lanelet.center_vertices:
                left_distances = [np.linalg.norm(center_point - left_point) for left_point in lanelet.left_vertices]
                min_left_distance = min(left_distances)

                right_distances = [np.linalg.norm(center_point - right_point) for right_point in lanelet.right_vertices]
                min_right_distance = min(right_distances)
                points.append({
                    'x_m': center_point[0],
                    'y_m': center_point[1],
                    'w_tr_right_m': min_right_distance,
                    'w_tr_left_m': min_left_distance
                })

        # Remove colliding points
        filtered_points = []
        last_point = points[0]
        filtered_points.append(last_point)
        deleted_points = []

        for i, point in enumerate(points[1:], start=1):
            distance = np.linalg.norm(
                np.array([point['x_m'], point['y_m']]) - np.array([last_point['x_m'], last_point['y_m']]))
            if distance > removing_distance:
                filtered_points.append(point)
                last_point = point
            else:
                deleted_points.append((i, point))


        npoints = np.asarray([points[0]['x_m'], points[0]['y_m'], points[0]['w_tr_right_m'], points[0]['w_tr_left_m']])

        for i, p in enumerate(points[1:]):
            new_p = np.asarray([points[i]['x_m'], points[i]['y_m'], points[i]['w_tr_right_m'], points[i]['w_tr_left_m']])
            npoints = np.vstack((npoints, new_p))


        # check if imported centerline should be flipped, i.e. reverse direction
        if flip_track:
            npoints = np.flipud(npoints)

        # check if imported centerline should be reordered for a new starting point
        if set_new_start:
            ind_start = np.argmin(np.power(npoints[:, 0] - new_start[0], 2)
                                  + np.power(npoints[:, 1] - new_start[1], 2))
            npoints = np.roll(npoints, npoints.shape[0] - ind_start, axis=0)


        # check minimum track width for vehicle width plus a small safety margin
        w_tr_min = np.amin(npoints[:, 2] + npoints[:, 3])
        if w_tr_min < vehicle_width + vehicle_safe_margin_m:
            warnings.warn(
                f"WARNING: Minimum track width {np.amin(w_tr_min)} is close to or smaller than vehicle width!"
            )

        return RaceTrack(
            x_m=npoints[:, 0],
            y_m=npoints[:, 1],
            w_tr_right_m=npoints[:, 2],
            w_tr_left_m=npoints[:, 3],
            lanelet_network=lanelet_network
        )

    @staticmethod
    def sort_lanelets_by_id(
            lanelet_network: LaneletNetwork
    ) -> List[Lanelet]:
        lanelets = lanelet_network.lanelets
        lanelets.sort(key=lambda l: int(l.lanelet_id))
        return lanelets

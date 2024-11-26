from dataclasses import dataclass
import warnings

from scipy.spatial.kdtree import KDTree
import numpy as np

# commonroad
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.lanelet import LaneletNetwork, Lanelet

# own code base
from commonroad_raceline_planner.utils.path_utils import calc_interpoint_length
from commonroad_raceline_planner.utils.exceptions import (
    TrackDataInvalidException,
)

# typing
from typing import Union, Optional, List


@dataclass
class RaceTrack:
    """
    Race track with centerline, left bounds and right bounds
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
        self.interpoint_length = calc_interpoint_length(x_m=self.x_m, y_m=self.y_m)
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
        if (
            not self.x_m.shape[0]
            == self.y_m.shape[0]
            == self.w_tr_left_m.shape[0]
            == self.w_tr_right_m.shape[0]
            == self.num_points
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
        return np.vstack((self.x_m, self.y_m, self.w_tr_right_m, self.w_tr_left_m))

    def to_2d_np_array(self) -> np.ndarray:
        """
        Convert to 2d numpy array
        :return: np.ndarray[x_m,y_m] -> dim = (num_points, 2)
        """
        return np.vstack((self.x_m, self.y_m))


class RaceTrackFactory:
    """
    Generates CR RaceTrack instance either from csv or CR scenario
    """

    @staticmethod
    def generate_racetrack_from_cr_scenario(
        lanelet_network: LaneletNetwork,
        planning_problem: PlanningProblem,
        vehicle_width: float,
        removing_distance: float = 0.5,
        vehicle_safe_margin_m: float = 0.5,
        lanelet_generation_loop_brake: int = 1000,
    ) -> RaceTrack:
        """
        Generates a cr racetrack object from cr scenario and planning problem
        :param lanelet_network: cr lanelet network
        :param planning_problem: cr planning problem
        :param vehicle_width: vehicle width
        :param removing_distance: remove close points in cr scenario with this distance
        :param vehicle_safe_margin_m: extra padding between vehicle and racetrack widtn
        :param lanelet_generation_loop_brake: after this many iterations lanelet track generation will through error
        :return: cr racetrack
        """
        # construct racetrack from lanelet network
        lanelets_ids_track: List[int] = []
        lanelet_id: int = lanelet_network.find_lanelet_by_position(
            [planning_problem.initial_state.position]
        )[0][0]
        lanelets_ids_track.append(lanelet_id)
        lanelet: Lanelet = lanelet_network.find_lanelet_by_id(lanelet_id)
        cnt: int = 0
        while len(lanelet.successor) > 0:
            if len(lanelet.successor) > 1:
                raise ValueError(
                    f"Lanelet {lanelet.lanelet_id} has {len(lanelet.successor)} successors, must only have one"
                )

            # break if closed loop encountered
            if lanelet.successor[0] == lanelets_ids_track[0]:
                break
            else:
                lanelets_ids_track.append(lanelet.successor[0])
                lanelet: Lanelet = lanelet_network.find_lanelet_by_id(
                    lanelet.successor[0]
                )

            cnt += 1
            if cnt > lanelet_generation_loop_brake:
                raise ValueError(
                    f"Stuck in lanelet generation loop for {lanelet_generation_loop_brake} iterations"
                )

        # Open the XML file and read the scenario
        points = []
        lanelets: List[Lanelet] = [
            lanelet_network.find_lanelet_by_id(lanelet_id)
            for lanelet_id in lanelets_ids_track
        ]

        for lanelet in lanelets:
            for center_point in lanelet.center_vertices:
                # for each point find closest left and right point
                left_distances = [
                    np.linalg.norm(center_point - left_point)
                    for left_point in lanelet.left_vertices
                ]
                min_left_distance = min(left_distances)

                right_distances = [
                    np.linalg.norm(center_point - right_point)
                    for right_point in lanelet.right_vertices
                ]
                min_right_distance = min(right_distances)
                points.append(
                    {
                        "x_m": center_point[0],
                        "y_m": center_point[1],
                        "w_tr_right_m": min_right_distance,
                        "w_tr_left_m": min_left_distance,
                    }
                )

        # Remove colliding points
        filtered_points = []
        last_point = points[0]
        filtered_points.append(last_point)
        deleted_points = []

        for i, point in enumerate(points[1:], start=1):
            distance = np.linalg.norm(
                np.array([point["x_m"], point["y_m"]])
                - np.array([last_point["x_m"], last_point["y_m"]])
            )
            if distance > removing_distance:
                filtered_points.append(point)
                last_point = point
            else:
                deleted_points.append((i, point))

        npoints = np.asarray(
            [
                points[0]["x_m"],
                points[0]["y_m"],
                points[0]["w_tr_right_m"],
                points[0]["w_tr_left_m"],
            ]
        )

        for i, p in enumerate(points[1:]):
            new_p = np.asarray(
                [
                    points[i]["x_m"],
                    points[i]["y_m"],
                    points[i]["w_tr_right_m"],
                    points[i]["w_tr_left_m"],
                ]
            )
            npoints = np.vstack((npoints, new_p))

        # Flip detection -> the algorithms assume that one drives with the clock, i.e. the right normals pointing inwards
        # if that is not true, flip the points
        if not RaceTrackFactory.check_clockwise(npoints[:, 0:2]):
            npoints = np.flipud(npoints)

        # set zero of centerline to beginning of planning problem
        kd_tree = KDTree(npoints[:, 0:2])
        _, idx = kd_tree.query(planning_problem.initial_state.position)
        npoints = np.roll(npoints, npoints.shape[0] - idx, axis=0)

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
            lanelet_network=lanelet_network,
        )

    @staticmethod
    def check_clockwise(points: np.ndarray) -> bool:
        """
        Checks if points are ordered clockwise.
        Resource:
        https://stackoverflow.com/questions/1165647/how-to-determine-if-a-list-of-polygon-points-are-in-clockwise-order
        :param points: (n,2) point array
        :return: True, if clockwise
        """
        edge_sum: float = 0
        for idx in range(points.shape[0] - 1):
            # (x2 - x1) * (y2 + y1)
            edge_sum += (points[idx + 1][0] - points[idx][0]) * (
                (points[idx + 1][1] + points[idx][1])
            )

        return True if edge_sum > 0 else False

    @staticmethod
    def _generate_racetrack_from_csv(
        file_path: str,
        vehicle_width: float,
        num_laps: int = 1,
        set_new_start: bool = False,
        new_start: Optional[np.ndarray] = None,
        vehicle_safe_margin_m: float = 0.5,
    ) -> RaceTrack:
        """
        FOR NEXT RELEASE: CURRENTLY NOT WORKING
        Import racetrack from csv

        Inputs:
        file_path:      file path of track.csv containing [x_m,y_m,w_tr_right_m,w_tr_left_m]
        imp_opts:       import options showing if a new starting point should be set or if the direction should be reversed
        width_veh:      vehicle width required to check against track width

        Outputs:
        race_track:   imported track [x_m, y_m, w_tr_right_m, w_tr_left_m]
        """

        raise NotImplementedError(
            "currently not implemented since no direct conversionto commonroad in cr designer"
        )

        # load data from csv file
        csv_data_temp = np.loadtxt(file_path, comments="#", delimiter=",")

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

        # Algorithms work for clockwise racetrack (i.e. they assume right bound points inward). If not given,
        # change order
        if not RaceTrackFactory.check_clockwise(reftrack_imp[:, :2]):
            reftrack_imp = np.flipud(reftrack_imp)

        # check if imported centerline should be reordered for a new starting point
        if set_new_start:
            ind_start = np.argmin(
                np.power(reftrack_imp[:, 0] - new_start[0], 2)
                + np.power(reftrack_imp[:, 1] - new_start[1], 2)
            )
            reftrack_imp = np.roll(
                reftrack_imp, reftrack_imp.shape[0] - ind_start, axis=0
            )

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
            w_tr_left_m=reftrack_imp[:, 3],
        )

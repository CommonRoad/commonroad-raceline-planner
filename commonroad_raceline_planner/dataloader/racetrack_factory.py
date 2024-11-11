import numpy as np


# commonroad
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer

from commonroad_raceline_planner.ractetrack import RaceTrack


class RaceTrackFactory:
    """
    Generates CR RaceTrack instance either from csv or CR scenario
    """

    @staticmethod
    def generate_racetrack_from_csv(
        file_path: str,
        imp_opts: dict,
        width_veh: float
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

        refline_ = np.tile(refline_, (imp_opts["num_laps"], 1))
        w_tr_r = np.tile(w_tr_r, imp_opts["num_laps"])
        w_tr_l = np.tile(w_tr_l, imp_opts["num_laps"])

        # assemble to a single array
        reftrack_imp = np.column_stack((refline_, w_tr_r, w_tr_l))

        # check if imported centerline should be flipped, i.e. reverse direction
        if imp_opts["flip_imp_track"]:
            reftrack_imp = np.flipud(reftrack_imp)

        # check if imported centerline should be reordered for a new starting point
        if imp_opts["set_new_start"]:
            ind_start = np.argmin(np.power(reftrack_imp[:, 0] - imp_opts["new_start"][0], 2)
                                  + np.power(reftrack_imp[:, 1] - imp_opts["new_start"][1], 2))
            reftrack_imp = np.roll(reftrack_imp, reftrack_imp.shape[0] - ind_start, axis=0)

        # check minimum track width for vehicle width plus a small safety margin
        w_tr_min = np.amin(reftrack_imp[:, 2] + reftrack_imp[:, 3])

        if w_tr_min < width_veh + 0.5:
            print("WARNING: Minimum track width %.2fm is close to or smaller than vehicle width!" % np.amin(w_tr_min))

        return RaceTrack(
            x_m=reftrack_imp[1],
            y_m=reftrack_imp[2],
            w_tr_right_m=reftrack_imp[3],
            w_tr_left_m=reftrack_imp[4]
        )

    def generate_racetrack_from_cr_scenario(
            self,
            file_path: str,
            imp_opts: dict,
            width_veh: float
    ):

        """
            This function converts a CommonRoad XML file to a CSV file.

            The CSV file contains the x and y coordinates of each point in the centerline of each lanelet in the lanelet network,
            as well as the distances from each center point to the nearest point in the left and right lines of the lanelet.
            (width_track_left and width_track_right)
            """
        # Open the XML file and read the scenario
        scenario, _ = CommonRoadFileReader(file_path).open()
        lanelet_network = scenario.lanelet_network

        points = []
        lanelets = RaceTrackFactory.sort_lanelets_by_id(lanelet_network)

        for lanelet in lanelets:
            if lanelet.predecessor and lanelet.successor:
                lanelet_id_str = str(lanelet.lanelet_id)
                print("lanelet" + lanelet_id_str + "has pred and suc")

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
        threshold_distance = 0.5  # Increased threshold for colliding points
        filtered_points = []
        last_point = points[0]
        filtered_points.append(last_point)
        deleted_points = []

        for i, point in enumerate(points[1:], start=1):
            distance = np.linalg.norm(
                np.array([point['x_m'], point['y_m']]) - np.array([last_point['x_m'], last_point['y_m']]))
            if distance > threshold_distance:
                filtered_points.append(point)
                last_point = point
            else:
                # # Average the colliding points
                # averaged_point = {
                #     'x_m': (point['x_m'] + last_point['x_m']) / 2,
                #     'y_m': (point['y_m'] + last_point['y_m']) / 2,
                #     'w_tr_right_m': (point['w_tr_right_m'] + last_point['w_tr_right_m']) / 2,
                #     'w_tr_left_m': (point['w_tr_left_m'] + last_point['w_tr_left_m']) / 2
                # }
                # filtered_points[-1] = averaged_point
                # last_point = averaged_point
                deleted_points.append((i, point))

        # Check if the filtered points form a closed map
        if np.linalg.norm(np.array([filtered_points[0]['x_m'], filtered_points[0]['y_m']]) - np.array(
                [filtered_points[-1]['x_m'], filtered_points[-1]['y_m']])) > threshold_distance:
            print("The map is not closed.")
        else:
            print("The map is closed.")

        # Print deleted points
        if deleted_points:
            print("The following points were deleted because they were too close to the previous point:")
            for index, point in deleted_points:
                print(f"Index: {index}, Point: {point}")
        else:
            print("No points were deleted.")

            return np.asarray(points)



    @staticmethod
    def sort_lanelets_by_id(lanelet_network):
        lanelets = lanelet_network.lanelets
        lanelets.sort(key=lambda l: int(l.lanelet_id))
        return lanelets




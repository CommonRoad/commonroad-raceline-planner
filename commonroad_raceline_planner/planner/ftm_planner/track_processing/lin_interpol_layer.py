import copy
import warnings
import math
import numpy as np

from commonroad_raceline_planner.planner.ftm_planner.track_processing.base_layer import (
    BaseRacetrackLayer,
)
from commonroad_raceline_planner.planner.ftm_planner.ftm_dto import DtoFTM


class LinearInterpolationLayer(BaseRacetrackLayer):
    """
    Linearly interpolate race track.
    """

    def linear_interpolate_racetrack(
        self,
        dto_racetrack: DtoFTM,
        interpol_stepsize: float,
        return_new_instance: bool,
        close_track: bool = True,
    ) -> DtoFTM:
        """
        Linearly interpolate race track and return new instance
        :param dto_racetrack: interpolated dto racetrack
        :param return_new_instance: deepcopies input and returns new instance
        :param interpol_stepsize: interpolation step size
        :return: interpolated dto racetrack
        """
        # create new instance or modify input
        if return_new_instance:
            interpolate_track = copy.deepcopy(dto_racetrack)
        else:
            interpolate_track = dto_racetrack

        # close racetrack
        if not interpolate_track.is_closed and close_track:
            interpolate_track.close_racetrack()
        elif not interpolate_track.is_closed and not close_track:
            warnings.warn("starting interpolation but race track is not closed")
        else:
            pass

        # interpolate
        if interpolate_track.is_interpolated:
            warnings.warn("racetrack is already interpolated wont interpolate again")
        else:
            # calculate desired lenghts depending on specified stepsize (+1 because last element is included)
            num_points: int = (
                math.ceil(interpolate_track.track_length / interpol_stepsize) + 1
            )
            interpoint_interpol_dist = np.linspace(
                start=0.0, stop=interpolate_track.track_length, num=num_points
            )

            # interpolate centerline and widths
            interpolate_track.x_m = np.interp(
                interpoint_interpol_dist,
                interpolate_track.track_length_per_point,
                interpolate_track.x_m,
            )
            interpolate_track.y_m = np.interp(
                interpoint_interpol_dist,
                interpolate_track.track_length_per_point,
                interpolate_track.y_m,
            )
            interpolate_track.w_tr_left_m = np.interp(
                interpoint_interpol_dist,
                interpolate_track.track_length_per_point,
                interpolate_track.w_tr_left_m,
            )
            interpolate_track.w_tr_right_m = np.interp(
                interpoint_interpol_dist,
                interpolate_track.track_length_per_point,
                interpolate_track.w_tr_right_m,
            )

            # TODO: make force update
            # recalc other members
            interpolate_track.num_points = interpolate_track.x_m.shape[0]
            interpolate_track.interpoint_length = np.sqrt(
                np.sum(
                    np.power(np.diff(interpolate_track.to_2d_np_array(), axis=0), 2),
                    axis=0,
                )
            )
            interpolate_track.track_length_per_point = np.cumsum(
                interpolate_track.interpoint_length
            )
            if interpolate_track.track_length_per_point[0] != 0.0:
                interpolate_track.track_length_per_point = np.insert(
                    interpolate_track.track_length_per_point, 0, 0.0
                )
            interpolate_track.track_length_per_point = np.insert(
                interpolate_track.track_length_per_point, 0, 0.0
            )
            interpolate_track.track_length = interpolate_track.track_length_per_point[
                -1
            ]

            # set flags
            interpolate_track.is_interpolated = True

        return interpolate_track

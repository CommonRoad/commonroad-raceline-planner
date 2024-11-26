import copy
import warnings

from commonroad_raceline_planner.planner.ftm_planner.track_processing.base_layer import (
    BaseRacetrackLayer,
)
from commonroad_raceline_planner.planner.ftm_planner.ftm_dto import DtoFTM


class WidthInflationLayer(BaseRacetrackLayer):
    """
    Inflates width of race track.
    """

    def inflate_width(
        self,
        dto_racetrack: DtoFTM,
        mininmum_track_width: float,
        return_new_instance: bool,
        close_track: bool = True,
    ) -> DtoFTM:
        """
        Inflates track boundaries to have minimum width
        """
        # create new instance or modify input
        if return_new_instance:
            inflated_track = copy.deepcopy(dto_racetrack)
        else:
            inflated_track = dto_racetrack

        # close racetrack
        if close_track and not inflated_track.is_closed:
            inflated_track.close_racetrack()
        elif not inflated_track.is_closed and not close_track:
            warnings.warn("starting inflating but race track is not closed")
        else:
            pass

        # ENFORCE MINIMUM TRACK WIDTH (INFLATE TIGHTER SECTIONS UNTIL REACHED)
        manipulation_flag: bool = False
        for i in range(inflated_track.x_m.shape[0]):
            cur_width: float = (
                inflated_track.w_tr_left_m[i] + inflated_track.w_tr_right_m[i]
            )

            if cur_width < mininmum_track_width:
                manipulation_flag = True

                # inflate to both sides equally
                inflated_track.w_tr_left_m[i] += (mininmum_track_width - cur_width) / 2
                inflated_track.w_tr_right_m[i] += (mininmum_track_width - cur_width) / 2

        if manipulation_flag:
            warnings.warn(
                "WARNING: Track region was smaller than requested minimum track width -> Applied artificial inflation in"
                " order to match the requirements!"
            )

        return inflated_track

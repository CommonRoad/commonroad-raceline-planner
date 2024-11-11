import numpy as np

from dataclasses import dataclass


from typing import List

@dataclass(frozen=True)
class RaceTrack:
    x_m: np.ndarray
    y_m: np.ndarray
    w_tr_right_m: np.ndarray
    w_tr_left_m: np.ndarray


"""
[x_m, y_m, w_tr_right_m, w_tr_left_m]"""
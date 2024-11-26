import numpy as np


def calc_interpoint_length(x_m: np.ndarray, y_m: np.ndarray) -> np.ndarray:
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

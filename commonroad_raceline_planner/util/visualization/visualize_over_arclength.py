import matplotlib.pyplot as plt
from commonroad_raceline_planner.raceline import RaceLine


# typing
from typing import List


def plot_trajectory_over_arclength(race_line: RaceLine) -> None:
    """
    plot raceline over arclength
    :param race_line: cr raceline
    """
    plt.title("velocity over arclength")
    plt.plot(race_line.length_per_point, race_line.velocity_long_per_point)
    plt.xlabel("Arc Length [m]")
    plt.ylabel("Velocity [m/s]")
    plt.show()

    plt.title("acceleration over arclength")
    plt.plot(race_line.length_per_point, race_line.acceleration_long_per_point)
    plt.xlabel("Arc Length [m]")
    plt.ylabel("Acceleration [m/s^2]")
    plt.show()

    plt.title("heading over arclength")
    plt.plot(race_line.length_per_point, race_line.heading_per_point)
    plt.xlabel("Arc Length [m]")
    plt.ylabel("Heading [rad]")
    plt.show()

    plt.title("curvature over arclength")
    plt.plot(race_line.length_per_point, race_line.curvature_per_point)
    plt.xlabel("Arc Length [m]")
    plt.ylabel("Curvature [1/m]")
    plt.show()

    plot_laptime(race_line=race_line)


def plot_laptime(race_line: RaceLine, vel_min_threshold: float = 0.2) -> None:
    """
    plot lap time
    :param race_line: cr race line
    """
    lap_time_per_point: List[float] = []
    sum_t = 0
    for idx in range(race_line.length_per_point.shape[0] - 1):
        # t = s/v
        s = race_line.length_per_point[idx + 1] - race_line.length_per_point[idx]
        v = race_line.velocity_long_per_point[idx]
        t = s / v if v > vel_min_threshold else 0.1
        sum_t += t
        lap_time_per_point.append(sum_t + t)

    plt.title("laptime over arclength")
    plt.plot(race_line.length_per_point[:-1], lap_time_per_point)
    plt.xlabel("Arc Length [m]")
    plt.ylabel("Lap Time (accumulated) [s]")
    plt.show()

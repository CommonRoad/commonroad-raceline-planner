import matplotlib.pyplot as plt
from commonroad_raceline_planner.raceline import RaceLine


def plot_trajectory_over_arclength(
        race_line: RaceLine
) -> None:
    """
    plot raceline over arclength
    :param race_line:
    :return:
    """
    plt.title("velocity over arclength")
    plt.plot(race_line.length_per_point, race_line.velocity_long_per_point)
    plt.show()

    plt.title("acceleration over arclength")
    plt.plot(race_line.length_per_point, race_line.acceleration_long_per_point)
    plt.show()

    plt.title("heading over arclength")
    plt.plot(race_line.length_per_point, race_line.heading_per_point)
    plt.show()

    plt.title("curvature over arclength")
    plt.plot(race_line.length_per_point, race_line.curvature_per_point)
    plt.show()


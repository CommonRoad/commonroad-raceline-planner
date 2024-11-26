import copy

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import rgb2hex, Normalize
from matplotlib import cm
from mpl_toolkits.axes_grid1 import make_axes_locatable

# commonroad
from commonroad.geometry.shape import Circle
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.lanelet import LaneletNetwork

# own code base
from commonroad_raceline_planner.raceline import RaceLine

# typing
from typing import List, Tuple

# cmap for coloring the velocity profile
cmap = cm.get_cmap("plasma")


def plot_param(
    lanelet_network: LaneletNetwork,
    planning_problem: PlanningProblem,
    race_line: RaceLine,
    title: str,
    param: np.ndarray,
    size_x: float = 10,
) -> None:
    """
    Plots a parameter of the raceline
    :param lanelet_network: cr lanelet network
    :param planning_problem: cr planning problem
    :param race_line: cr raceline
    :param title: title string of plot
    :param param: param as (n,) np.ndarray
    :param size_x: size of plot
    """
    _ = plt.figure(figsize=(40, 20))

    # get plot limits from reference path
    plot_limits: List[float] = obtain_plot_limits_from_reference_path(
        race_line.points, margin=20
    )
    ratio_x_y = (plot_limits[1] - plot_limits[0]) / (plot_limits[3] - plot_limits[2])

    renderer = MPRenderer(plot_limits=plot_limits, figsize=(size_x, size_x / ratio_x_y))
    renderer.draw_params.dynamic_obstacle.draw_icon = True

    lanelet_network.draw(renderer)
    planning_problem.draw(renderer)

    param_min = np.min(param)
    param_max = np.max(param)

    for idx in range(race_line.points.shape[0]):
        draw_trajectory_positions(
            renderer,
            reference_point=race_line.points[idx],
            velocity=param[idx],
            v_min=param_min,
            v_max=param_max,
        )

    # draw scenario and renderer
    renderer.render()

    # plot legend before divider so they appear nice
    plt.xlabel("East [m]")
    plt.ylabel("North [m]")

    # colorbar work-around
    divider = make_axes_locatable(plt.gca())
    cax = divider.append_axes("right", size="5%", pad=0.05)
    norm = Normalize(vmin=param_min, vmax=param_max)
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    plt.colorbar(sm, cax=cax, cmap="plasma", orientation="vertical")
    plt.title(title)
    plt.show()


def plot_trajectory_with_all_quantities(
    lanelet_network: LaneletNetwork,
    planning_problem: PlanningProblem,
    race_line: RaceLine,
    size_x: float = 10,
) -> None:
    """
    Plot all quantities
    Plots a parameter of the raceline
    :param lanelet_network: cr lanelet network
    :param planning_problem: cr planning problem
    :param race_line: cr raceline
    :param size_x: size of plot
    """
    plot_trajectory_with_velocity(
        lanelet_network=lanelet_network,
        planning_problem=planning_problem,
        race_line=race_line,
        size_x=size_x,
    )
    plot_trajectory_with_acceleration(
        lanelet_network=lanelet_network,
        planning_problem=planning_problem,
        race_line=race_line,
        size_x=size_x,
    )
    plot_trajectory_with_curvature(
        lanelet_network=lanelet_network,
        planning_problem=planning_problem,
        race_line=race_line,
        size_x=size_x,
    )
    plot_trajectory_with_heading(
        lanelet_network=lanelet_network,
        planning_problem=planning_problem,
        race_line=race_line,
        size_x=size_x,
    )


def plot_trajectory_with_velocity(
    lanelet_network: LaneletNetwork,
    planning_problem: PlanningProblem,
    race_line: RaceLine,
    size_x: float = 10,
) -> None:
    """
    Plots trajectory with velocity
    :param lanelet_network: cr lanelet network
    :param planning_problem: cr planning problem
    :param race_line: cr raceline
    :param size_x: size of plot
    """
    plot_param(
        lanelet_network=lanelet_network,
        planning_problem=planning_problem,
        race_line=race_line,
        param=race_line.velocity_long_per_point,
        size_x=size_x,
        title="Velocity [m/s]",
    )


def plot_trajectory_with_curvature(
    lanelet_network: LaneletNetwork,
    planning_problem: PlanningProblem,
    race_line: RaceLine,
    size_x: float = 10,
) -> None:
    """
    Plots trajectory with curvature
    :param lanelet_network: cr lanelet network
    :param planning_problem: cr planning problem
    :param race_line: cr raceline
    :param size_x: size of plot
    """
    plot_param(
        lanelet_network=lanelet_network,
        planning_problem=planning_problem,
        race_line=race_line,
        param=race_line.curvature_per_point,
        size_x=size_x,
        title="Curvature [1/m]",
    )


def plot_trajectory_with_acceleration(
    lanelet_network: LaneletNetwork,
    planning_problem: PlanningProblem,
    race_line: RaceLine,
    size_x: float = 10,
) -> None:
    """
    Plots trajectory with acceleration
    :param lanelet_network: cr lanelet network
    :param planning_problem: cr planning problem
    :param race_line: cr raceline
    :param size_x: size of plot
    """
    plot_param(
        lanelet_network=lanelet_network,
        planning_problem=planning_problem,
        race_line=race_line,
        param=race_line.acceleration_long_per_point,
        size_x=size_x,
        title="Acceleration [m/s^2]",
    )


def plot_trajectory_with_heading(
    lanelet_network: LaneletNetwork,
    planning_problem: PlanningProblem,
    race_line: RaceLine,
    size_x: float = 10,
) -> None:
    """
    Plots trajectory with heading
    :param lanelet_network: cr lanelet network
    :param planning_problem: cr planning problem
    :param race_line: cr raceline
    :param size_x: size of plot
    """
    plot_param(
        lanelet_network=lanelet_network,
        planning_problem=planning_problem,
        race_line=race_line,
        param=race_line.heading_per_point,
        size_x=size_x,
        title="Heading [rad]",
    )


def draw_trajectory_positions(
    renderer: MPRenderer,
    reference_point: np.ndarray,
    velocity: float,
    v_min: float,
    v_max: float,
    point_radius: float = 0.5,
) -> None:
    """
    Draws reference_path state
    :param renderer: cr MPRenderer
    :param reference_point: point to draw as (2,) np.ndarry
    :param velocity: velocity of point
    :param v_min: v_min
    :param v_max: v_max
    :param point_radius: radius to display point
    """
    normalized_velocity: float = (
        (velocity - v_min) / (v_max - v_min) if not np.isclose(v_max, v_min) else 0
    )
    rbg_color = cmap(normalized_velocity)
    hex_color = rgb2hex(rbg_color)
    draw_params = copy.copy(renderer.draw_params)
    draw_params.shape.facecolor = hex_color
    draw_params.shape.edgecolor = hex_color

    occ_pos = Circle(radius=point_radius, center=reference_point)
    occ_pos.draw(renderer, draw_params=draw_params)


def get_velocity_min_max_from_trajectory(
    velocity_profile: np.ndarray,
) -> Tuple[float, float]:
    """
    Gets min and max velocity from global trajectory for color coding.
    :param velocity_profile: velocity profile as (n,2) np.ndarray
    :return: tuple[v_min, v_max]
    """
    min_velocity: float = np.min(velocity_profile)
    max_velocity: float = np.max(velocity_profile)
    return (min_velocity, max_velocity)


def obtain_plot_limits_from_reference_path(
    reference_path: np.ndarray, margin: float = 10.0
) -> List[int]:
    """
    Obtrains plot limits from reference path
    :param reference_path: reference path (2,) np.ndarray
    :return: list [xmin, xmax, ymin, xmax] of plot limits
    """
    x_min = min(reference_path[:, 0])
    x_max = max(reference_path[:, 0])
    y_min = min(reference_path[:, 1])
    y_max = max(reference_path[:, 1])

    plot_limits = [x_min - margin, x_max + margin, y_min - margin, y_max + margin]

    return plot_limits

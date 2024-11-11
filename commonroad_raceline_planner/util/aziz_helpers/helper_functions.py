import json
from PIL import Image
import csv
import numpy as np
import matplotlib.pyplot as plt

import os
import subprocess
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer


def sort_lanelets_by_id(lanelet_network):
    lanelets = lanelet_network.lanelets
    lanelets.sort(key=lambda l: int(l.lanelet_id))
    return lanelets
def generate_csv_from_xml(xml_file_path, csv_file_path='inputs/tracks/lanelet_data.csv'):
    """
    This function converts a CommonRoad XML file to a CSV file.

    The CSV file contains the x and y coordinates of each point in the centerline of each lanelet in the lanelet network,
    as well as the distances from each center point to the nearest point in the left and right lines of the lanelet.
    (width_track_left and width_track_right)
    """
    # Open the XML file and read the scenario
    scenario, _ = CommonRoadFileReader(xml_file_path).open()
    lanelet_network = scenario.lanelet_network

    points = []
    lanelets = sort_lanelets_by_id(lanelet_network)

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

    with open(csv_file_path, 'w', newline='') as csvfile:
        fieldnames = ['x_m', 'y_m', 'w_tr_right_m', 'w_tr_left_m']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        writer.writeheader()

        for point in filtered_points:
            writer.writerow(point)

    print(f"CSV file has been created at {csv_file_path}")

    # Print deleted points
    if deleted_points:
        print("The following points were deleted because they were too close to the previous point:")
        for index, point in deleted_points:
            print(f"Index: {index}, Point: {point}")
    else:
        print("No points were deleted.")

    # Plotting the filtered points to visualize
    x_coords = [p['x_m'] for p in filtered_points]
    y_coords = [p['y_m'] for p in filtered_points]

    plt.figure(figsize=(16, 10))

    rnd = MPRenderer()

    # Draw the scenario and planning problem set on the same axis
    scenario.draw(rnd)
    rnd.render()

    # Plot the centerline points on top of the scenario
    plt.scatter(x_coords, y_coords, color='red', label='Centerline', zorder=10)
    plt.title('Filtered Centerline')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.axis('equal')
    plt.savefig('/home/bouzirim/Pictures/evaluation figures/svg/filtered_centerline.svg',format="svg")
    plt.show()


# Example usage:
# generate_csv_from_xml('path_to_your_xml_file.xml')


def add_to_dict(dictionary, key, value):
    dictionary[key] = value


def resize_images_in_directory(directory: str, target_width: int, target_height: int):
    for filename in os.listdir(directory):
        if filename.endswith(".png"):
            file_path = os.path.join(directory, filename)
            with Image.open(file_path) as img:
                resized_img = img.resize((target_width, target_height), Image.LANCZOS)
                resized_img.save(file_path)


def plot_reference_path(reference_path_polyline, scenario, pp):
    # Extract x and y coordinates
    x_coords = [point[0] for point in reference_path_polyline]
    y_coords = [point[1] for point in reference_path_polyline]

    # Plot the path
    plt.figure(figsize=(12, 8))

    rnd = MPRenderer()

    # Draw the scenario and planning problem set on the same axis
    scenario.draw(rnd)
    pp.draw(rnd)
    rnd.render()

    plt.plot(x_coords, y_coords, marker='o', linestyle='-', color='b', label='Reference Path', zorder=10, markersize=2)
    # Add labels and title
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Reference Path Polyline and Scenario')
    plt.grid(True)
    plt.show()


def plot_error_curvature_gp_dc(raceline_planner, reactive_planner):
    # Extracting position and curvature from raceline planner
    pos = raceline_planner.trajectory_opt[:, 0]
    curv = raceline_planner.trajectory_opt[:, 4]
    gp_array = np.column_stack((pos, curv))

    # Extracting position and curvature from reactive planner
    dc_array = np.column_stack(
        (reactive_planner.coordinate_system.ref_pos, reactive_planner.coordinate_system.ref_curv))

    # Calculate the error between the curvature values of the two planners
    curvature_error = []
    for ref_pos, ref_curv in dc_array:
        # Find the closest position in gp_array
        closest_idx = (np.abs(gp_array[:, 0] - ref_pos)).argmin()
        closest_curv = gp_array[closest_idx, 1]
        curvature_error.append(closest_curv - ref_curv)

    curvature_error = np.array(curvature_error)

    # Plotting the error
    plt.figure(figsize=(10, 6))
    plt.plot(dc_array[:, 0], curvature_error, label='Curvature Error (planned vs Drivability Check)')
    plt.xlabel('Position (m)')
    plt.ylabel('Curvature Error (rad/m)')
    plt.title('Error Between Curvature Profiles (planned vs Drivability Check)')
    plt.legend()
    plt.grid(True)
    plt.savefig('/home/bouzirim/Pictures/evaluation figures/svg/error_curvature_mc.svg',format="svg")
    plt.show()

def save_velocity_for_boxplot(state_list):
    velocities = [state.velocity for state in state_list]
    import json

    with open('outputs/box_plot/velocities_mc.json', 'w') as json_file:
        json.dump(velocities, json_file)

def save_yaw_rate_for_boxplot(state_list):
    yaw_rates = [state.yaw_rate for state in state_list]
    import json

    with open('outputs/box_plot/yaw_rates_sp.json', 'w') as json_file:
        json.dump(yaw_rates, json_file)

def plot_velocity_profiles(sp_file_path, mc_file_path):
    """
    Function to generate a box plot comparing velocity profiles from
    the shortest path planner and the minimum curvature planner with different colors and a legend.

    Parameters:
    sp_file_path (str): Path to the JSON file containing shortest path planner velocities.
    mc_file_path (str): Path to the JSON file containing minimum curvature planner velocities.
    """
    # Load the velocity data from the JSON files
    with open(sp_file_path, 'r') as sp_file:
        velocities_sp = json.load(sp_file)

    with open(mc_file_path, 'r') as mc_file:
        velocities_mc = json.load(mc_file)

    # Prepare data for the boxplot
    data = [velocities_sp, velocities_mc]

    # Create the boxplot with custom colors
    fig, ax = plt.subplots(figsize=(8, 6))
    box = ax.boxplot(data, patch_artist=True, labels=['Shortest Path Planner', 'Minimum Curvature Planner'])

    # Define custom colors
    colors = ['lightblue', 'lightgreen']

    # Color the boxes
    for patch, color in zip(box['boxes'], colors):
        patch.set_facecolor(color)

    # Set titles and labels
    ax.set_title('Velocity Profiles Comparison')
    ax.set_ylabel('Velocity')

    # Add a legend for the box colors
    plt.legend([box["boxes"][0], box["boxes"][1]], ['Shortest Path Planner', 'Minimum Curvature Planner'], loc='upper right')

    # Show the plot
    plt.grid(False)
    plt.savefig('/home/bouzirim/Pictures/evaluation figures/svg/velocity_profile_comparison.svg',format="svg")
    plt.show()

def plot_yaw_rate_profiles(sp_file_path, mc_file_path):
    """
    Function to generate a box plot comparing yaw rate profiles from
    the shortest path planner and the minimum curvature planner with different colors and a legend.

    Parameters:
    sp_file_path (str): Path to the JSON file containing shortest path planner yaw rates.
    mc_file_path (str): Path to the JSON file containing minimum curvature planner yaw rates.
    """
    # Load the yaw rate data from the JSON files
    with open(sp_file_path, 'r') as sp_file:
        yaw_rates_sp = json.load(sp_file)

    with open(mc_file_path, 'r') as mc_file:
        yaw_rates_mc = json.load(mc_file)

    # Prepare data for the boxplot
    data = [yaw_rates_sp, yaw_rates_mc]

    # Create the boxplot with custom colors
    fig, ax = plt.subplots(figsize=(8, 6))
    box = ax.boxplot(data, patch_artist=True, labels=['Shortest Path Planner', 'Minimum Curvature Planner'])

    # Define custom colors
    colors = ['lightblue', 'lightgreen']

    # Color the boxes
    for patch, color in zip(box['boxes'], colors):
        patch.set_facecolor(color)

    # Set titles and labels
    ax.set_title('Yaw Rate Profiles Comparison (Steering angle rate)')
    ax.set_ylabel('Yaw Rate')

    # Add a legend for the box colors
    plt.legend([box["boxes"][0], box["boxes"][1]], ['Shortest Path Planner', 'Minimum Curvature Planner'], loc='upper right')

    # Show the plot
    plt.grid(False)
    plt.show()





def convert_png_to_svg_embed(png_path, svg_path=None):
    """
    Embed a PNG image into an SVG file for easy referencing.

    Parameters:
    png_path (str): Path to the input PNG file.
    svg_path (str, optional): Path to the output SVG file. If not provided, it will save in the same directory as the PNG.

    Returns:
    str: The output SVG file path.
    """
    # Check if the input PNG file exists
    if not os.path.exists(png_path):
        raise FileNotFoundError(f"The input PNG file '{png_path}' does not exist.")

    # Set the output SVG path if not provided
    if svg_path is None:
        base_name = os.path.splitext(png_path)[0]
        svg_path = f"{base_name}.svg"

    # Use Inkscape to embed the PNG in an SVG wrapper
    command = [
        "inkscape",
        "--export-filename", svg_path,
        "--export-type=svg",
        png_path
    ]
    subprocess.run(command, check=True)

    print(f"SVG successfully created: {svg_path}")
    return svg_path

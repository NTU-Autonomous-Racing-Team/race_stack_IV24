import os
from skimage.morphology import skeletonize
import matplotlib.pyplot as plt
import yaml
from scipy.ndimage import distance_transform_edt as edt
from PIL import Image
import sys
import numpy as np
import pandas as pd
from typing import Tuple
import subprocess

import argparse

# adapted from https://github.com/CL2-UWaterloo/Head-to-Head-Autonomous-Racing/blob/main/gym/f110_gym/envs/laser_models.py
def get_centerline(map_name: str, track_width_margin: float) -> str:
    """
    use the map generated using SLAM_toolbox and generate the cetnerlines of the track.

    Parameters:
        - map_name: Map file name.
        - track_width_margin: Track safety margin.
    """

    map_img_path = ''
    for ext in ['.png', '.pgm']:
        if os.path.exists(f"maps/{map_name}{ext}"):
            map_img_path = f"maps/{map_name}{ext}"
            break
    else:
        raise FileNotFoundError("Map not found!")

    map_yaml_path = f"maps/{map_name}.yaml"

    # process map image
    raw_map_img = np.array(Image.open(map_img_path).transpose(Image.FLIP_TOP_BOTTOM)).astype(np.float64)
    map_img = raw_map_img.copy()
    map_img[map_img <= 210.] = 0
    map_img[map_img > 210.] = 1

    # compute distance transform
    dist_transform = edt(map_img)  # eculidan distance

    # skeletonize image map
    THRESHOLD = 0.17  # TODO: PARAMETER
    centers = dist_transform > THRESHOLD * dist_transform.max()
    centerline = skeletonize(centers)

    # extract cetnerline coordinate
    map_height = map_img.shape[0]
    LEFT_START_Y = map_height // 2 - 120
    NON_EDGE = 0.0
    left_start_y = LEFT_START_Y
    left_start_x = 0
    while centerline[left_start_y][left_start_x] == NON_EDGE:
        left_start_x += 1

    sys.setrecursionlimit(20000)
    visited = set()
    centerline_points = []
    track_widths = []
    DIRECTIONS = [(0, -1), (-1, 0), (0, 1), (1, 0), (-1, 1), (-1, -1), (1, 1), (1, -1)]

    print("get_centerline SUCCESS")

    def dfs(point: Tuple):
        if point in visited:
            return
        visited.add(point)
        centerline_points.append(np.array(point))
        track_widths.append(np.array([dist_transform[point[1]][point[0]], dist_transform[point[1]][point[0]]]))

        for dx, dy in DIRECTIONS:
            nx, ny = point[0] + dx, point[1] + dy
            if 0 <= ny < dist_transform.shape[0] and 0 <= nx < dist_transform.shape[1] and centerline[ny][
                nx] != NON_EDGE and (nx, ny) not in visited:
                dfs((nx, ny))

    dfs((left_start_x, left_start_y))

    #convert to numpy arrays for easier manipulation
    waypoints = np.array(centerline_points)
    track_widths_np = np.array(track_widths)

    # load map metadata
    with open(map_yaml_path, 'r') as yaml_stream:
        map_metadata = yaml.safe_load(yaml_stream)
        map_resolution = map_metadata['resolution']
        origin = map_metadata['origin']

    orig_x, orig_y = origin[:2]
    transformed_data = np.concatenate((waypoints, track_widths_np), axis=1) * map_resolution
    transformed_data += np.array([orig_x, orig_y, 0, 0])
    transformed_data -= np.array([0, 0, track_width_margin, track_width_margin])

    # save to CSV
    csv_path = f"inputs/tracks/{map_name}.csv"
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    with open(csv_path, 'w') as fh:
        fh.write("# x_m,y_m,w_tr_right_m,w_tr_left_m\n")
        np.savetxt(fh, transformed_data, fmt='%0.4f', delimiter=',', comments='')
    return csv_path


def test_centerline(map_name: str):
    """
    test and verify get_centerline map is correct.
    """
    if os.path.exists(f"maps/{map_name}.png"):
        map_img_path = f"maps/{map_name}.png"
    elif os.path.exists(f"maps/{map_name}.pgm"):
        map_img_path = f"maps/{map_name}.pgm"
    else:
        raise Exception("Map not found!")

    map_yaml_path = f"maps/{map_name}.yaml"

    # Load the map image and metadata
    map_img = np.array(Image.open(map_img_path).transpose(Image.FLIP_TOP_BOTTOM))
    map_img = map_img.astype(np.float64)

    with open(map_yaml_path, 'r') as yaml_stream:
        try:
            map_metadata = yaml.safe_load(yaml_stream)
            map_resolution = map_metadata['resolution']
            origin = map_metadata['origin']
        except yaml.YAMLError as ex:
            print(ex)

    orig_x = origin[0]
    orig_y = origin[1]
    orig_s = np.sin(origin[2])
    orig_c = np.cos(origin[2])

    # Load the track data
    raw_data = pd.read_csv(f"inputs/tracks/{map_name}.csv", comment='#')
    x = raw_data.iloc[:, 0].values
    y = raw_data.iloc[:, 1].values
    wr = raw_data.iloc[:, 2].values
    wl = raw_data.iloc[:, 3].values

    x -= orig_x
    y -= orig_y

    x /= map_resolution
    y /= map_resolution

    # Plot the data for sanity check
    plt.figure(figsize=(10, 10))
    plt.imshow(map_img, cmap='gray', origin='lower')
    plt.plot(x, y, 'r-', label='Centerline')
    plt.plot(x + wr, y + wl, 'b-', label='Right Track Boundary')
    plt.plot(x - wr, y - wl, 'g-', label='Left Track Boundary')
    plt.legend()
    plt.show()

    print("test_centerline SUCCESS")


if __name__ == "__main__":

    # parser = argparse.ArgumentParser(description="Generate waypoints for optimized trajectory given reference track.")
    # parser.add_argument('--name', type=str, help='Map name for generating waypoints of optimized trajectory', default='singapore_map')
    # args = parser.parse_args()
    # MAP_NAME = args.name
    MAP_NAME = "test_map"

    # try:
    print("get_centerline started.")
    get_centerline(MAP_NAME, track_width_margin=0.0)

    # print("test_centerline started.")
    test_centerline(MAP_NAME)

    print("generate friction map started")
    subprocess.run(['python3', 'main_gen_frictionmap.py'])

    print("trajectory_optimization started")
    subprocess.run(['python3', 'main_globaltraj.py'])

    # print("WAYPOINT GENERATION COMPLETE")
    # # except:
    # #     print("An error has occured.")
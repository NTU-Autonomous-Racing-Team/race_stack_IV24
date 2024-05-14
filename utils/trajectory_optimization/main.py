import os
import subprocess
from typing import Tuple
import sys
import argparse

from skimage.morphology import skeletonize
import scipy
from scipy.ndimage import distance_transform_edt as edt
import numpy as np
import matplotlib.pyplot as plt
import yaml
from PIL import Image
import os
import pandas as pd


# adapted from https://github.com/CL2-UWaterloo/Head-to-Head-Autonomous-Racing/blob/main/gym/f110_gym/envs/laser_models.py
def get_centerline(map_name: str, track_width_margin: float) -> str:
    """
    use the map generated using SLAM_toolbox and generate the cetnerlines of the track.

    Parameters:
        - map_name: Map file name.
        - track_width_margin: Track safety margin.
    """

    # get map path
    map_img_path = ''
    for ext in ['.png', '.pgm']:
        if os.path.exists(f"maps/{map_name}{ext}"):
            map_img_path = f"maps/{map_name}{ext}"
            break
    else:
        raise FileNotFoundError("Map not found.")

    map_yaml_path = f"maps/{map_name}.yaml"
    raw_map_img = np.array(Image.open(map_img_path).transpose(Image.FLIP_TOP_BOTTOM))
    raw_map_img = raw_map_img.astype(np.float64)



    plt.figure(figsize=(10, 10))
    plt.imshow(raw_map_img, cmap='gray', origin='lower')

    map_img = raw_map_img.copy()
    map_img[map_img <= 210.] = 0
    map_img[map_img > 210.] = 1

    map_height = map_img.shape[0]

    plt.figure(figsize=(10, 10))
    plt.imshow(map_img, cmap='gray', origin='lower')

    plt.figure(figsize=(10, 10))
    dist_transform = scipy.ndimage.distance_transform_edt(map_img)
    plt.imshow(dist_transform, cmap='gray', origin='lower')

    THRESHOLD = 0.17
    centers = dist_transform > THRESHOLD * dist_transform.max()
    plt.figure(figsize=(10, 10))
    plt.imshow(centers, origin='lower', cmap='gray')

    plt.figure(figsize=(10, 10))
    centerline = skeletonize(centers)
    plt.imshow(centerline, origin='lower', cmap='gray')

    plt.figure(figsize=(10, 10))
    centerline_dist = np.where(centerline, dist_transform, 0)
    plt.imshow(centerline_dist, origin='lower', cmap='gray')

    LEFT_START_Y = map_height // 2 - 120

    NON_EDGE = 0.0
    # Use DFS to extract the outer edge
    left_start_y = LEFT_START_Y
    left_start_x = 0
    while (centerline_dist[left_start_y][left_start_x] == NON_EDGE):
        left_start_x += 1

    print(f"Starting position for left edge: {left_start_x} {left_start_y}")

    sys.setrecursionlimit(20000)

    visited = {}
    centerline_points = []
    track_widths = []
    # DIRECTIONS = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    # If you want the other direction first
    DIRECTIONS = [(0, -1), (-1, 0), (0, 1), (1, 0), (-1, 1), (-1, -1), (1, 1), (1, -1)]

    starting_point = (left_start_x, left_start_y)

    def dfs(point):
        if (point in visited): return
        visited[point] = True
        centerline_points.append(np.array(point))
        track_widths.append(np.array([centerline_dist[point[1]][point[0]], centerline_dist[point[1]][point[0]]]))

        for direction in DIRECTIONS:
            if (centerline_dist[point[1] + direction[1]][point[0] + direction[0]] != NON_EDGE and (
            point[0] + direction[0], point[1] + direction[1]) not in visited):
                dfs((point[0] + direction[0], point[1] + direction[1]))

    dfs(starting_point)

    fig, (ax1, ax2, ax3, ax4) = plt.subplots(1, 4, figsize=(20, 5))
    ax1.axis('off')
    ax2.axis('off')
    ax3.axis('off')
    ax4.axis('off')

    centerline_img = np.zeros(map_img.shape)
    for x, y in centerline_points[:len(centerline_points) // 10]:
        centerline_img[y][x] = 255
    ax1.imshow(centerline_img, cmap='Greys', vmax=1, origin='lower')
    ax1.set_title("First 10% points")

    centerline_img = np.zeros(map_img.shape)
    for x, y in centerline_points[:len(centerline_points) // 4]:
        centerline_img[y][x] = 255
    ax2.imshow(centerline_img, cmap='Greys', vmax=1, origin='lower')
    ax2.set_title("First 25% points")

    centerline_img = np.zeros(map_img.shape)
    for x, y in centerline_points[:int(len(centerline_points) / 1.4)]:
        centerline_img[y][x] = 255
    ax3.imshow(centerline_img, cmap='Greys', vmax=1, origin='lower')
    ax3.set_title("First 50% points")

    centerline_img = np.zeros(map_img.shape)
    for x, y in centerline_points:
        centerline_img[y][x] = 1000
    ax4.imshow(centerline_img, cmap='Greys', vmax=1, origin='lower')
    ax4.set_title("All points")
    fig.tight_layout()

    track_widths_np = np.array(track_widths)
    waypoints = np.array(centerline_points)
    print(f"Track widths shape: {track_widths_np.shape}, waypoints shape: {waypoints.shape}")

    data = np.concatenate((waypoints, track_widths_np), axis=1)
    print(data.shape)

    with open(map_yaml_path, 'r') as yaml_stream:
        try:
            map_metadata = yaml.safe_load(yaml_stream)
            map_resolution = map_metadata['resolution']
            origin = map_metadata['origin']
        except yaml.YAMLError as ex:
            print(ex)

    # calculate map parameters
    orig_x = origin[0]
    orig_y = origin[1]
    orig_s = np.sin(origin[2])
    orig_c = np.cos(origin[2])

    # get the distance transform
    transformed_data = data
    transformed_data *= map_resolution
    transformed_data += np.array([orig_x, orig_y, 0, 0])

    # Safety margin
    transformed_data -= np.array([0, 0, track_width_margin, track_width_margin])

    with open(f"inputs/tracks/{map_name}.csv", 'wb') as fh:
        np.savetxt(fh, transformed_data, fmt='%0.4f', delimiter=',', header='x_m,y_m,w_tr_right_m,w_tr_left_m')

    print("COMPLETE")

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
    raw_data = pd.read_csv(f"inputs/tracks/{map_name}.csv")
    x = raw_data["# x_m"].values
    y = raw_data["y_m"].values
    wr = raw_data["w_tr_right_m"].values
    wl = raw_data["w_tr_left_m"].values

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

def main():
    parser = argparse.ArgumentParser(description='Map name to access.')
    parser.add_argument('map_name', nargs='?', default='HESL', help='the name of the map to process')
    args = parser.parse_args()
    print(f"Map name is: {args.map_name}") 

    MAP_NAME = args.map_name

    get_centerline(MAP_NAME, track_width_margin=0.0)
    test_centerline(MAP_NAME)
    # subprocess.run(['python3', 'main_globaltraj.py', '--name', MAP_NAME])
    
if __name__ == "__main__":
   main() 
    

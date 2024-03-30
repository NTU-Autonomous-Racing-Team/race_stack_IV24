import rclpy
from rclpy.node import Node

import numpy as np
from skimage.morphology import skeletonize
import matplotlib.pyplot as plt
import yaml
import scipy
from scipy.ndimage import distance_transform_edt as edt
from PIL import Image
import os

class GlobalWaypoints:
    def __init__(self):
        super().__init__('global_waypoints_node')

    def get_centerline (self, map_name : str):
        """
        use the map generated using SLAM_toolbox and generate the cetnerlines of the track.
        """




        pass
    def test_centerline(self):
        """
        test and verify get_centerline map is correct.
        """
        pass

    def get_trajectory_generation(self):
        """
        generate waypoints for publishing to pure pursuit.
        """
        pass



def main():
    rclpy.init()
    global_waypoints_node = GlobalWaypoints()
    rclpy.spin(global_waypoints_node)

    global_waypoints_node.destroy_node()
    rclpy.shutdown()
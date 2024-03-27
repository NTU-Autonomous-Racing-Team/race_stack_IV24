#!/usr/bin/env python3

import csv
import numpy as np

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class WaypointVisualizer(Node):

    def __init__(self):
        super().__init__('waypoint_visualizer')
        
        self.waypoint_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        
        coordinate_arr = np.zeros((0, 2))
        with open("waypoint_data.csv", newline="") as f_in:
            reader = csv.reader(f_in)
            for row in reader:
                print(row)
                coordinate_arr = np.vstack([coordinate_arr, [float(row[0]), float(row[1])]])
        
        scale = 0.2
        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.ns = "waypoints"
        self.marker.id = 1
        self.marker.type = Marker.SPHERE_LIST  # Custom self.marker type
        self.marker.action = Marker.ADD # coordinate_arr        
        self.marker.scale.x = scale
        self.marker.scale.y = scale
        self.marker.scale.z = scale
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 0.0
        self.marker.color.b = 1.0
        for i in range(0, len(coordinate_arr)):
            points = Point()
            points.x = coordinate_arr[i][0]
            points.y = coordinate_arr[i][1]
            points.z = 0.0
            self.marker.points.append(points)

        timer_period = 5.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        self.waypoint_publisher.publish(self.marker)

def main(args=None):
    rclpy.init(args=args)

    waypoint_visualizer = WaypointVisualizer()

    rclpy.spin(waypoint_visualizer)

    waypoint_visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

import math


class Overtake(Node):
    def __init__(self):
        super().__init__("overtake")

        # Subscribing to relevant topics
        self.sub_scan = self.create_subscription(
            LaserScan, "scan", self.scan_callback, 10
        )

        self.distance_threshold = 0.2

    def scan_callback(self, scan_data):
        ranges = scan_data.ranges
        angle_increment = scan_data.angle_increment
        angle_min = scan_data.angle_min
        angle_max = scan_data.angle_max

        found_start = False
        found_end = False
        one_iteration = False
        delta = float("inf")
        line_length = 0.0
        start_of_car = None
        end_of_car = None

        # Loop through the ranges and find when there is a big jump in the distance
        for i in range(len(ranges)):

            delta = ranges[i] - ranges[i - 1]

            if delta < self.distance_threshold and found_start != True:
                start_of_car = i
                found_start = True
                one_iteration = True
            if delta > self.distance_threshold and found_start == True:
                end_of_car = i - 1
                found_end = True
            # print(start_of_car, end_of_car)
            print(found_end)
            if found_start == True and found_end == False and one_iteration == True:
                a = ranges[i]
                b = ranges[i - 1]
                line_length += math.sqrt(
                    a**2 + b**2 - 2 * a * b * math.cos(angle_increment)
                )
                if line_length < 0.3:
                    car_detected = True

            if car_detected == True:
                break


def main(args=None):

    rclpy.init(args=args)

    overtake = Overtake()

    rclpy.spin(overtake)

    overtake.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":

    main()

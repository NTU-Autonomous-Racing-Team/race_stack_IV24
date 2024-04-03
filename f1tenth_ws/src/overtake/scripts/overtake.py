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
            LaserScan, "scan", self.scan_callback, 1
        )

        self.distance_threshold = 0.7
        self.distance_threshold_car = 0.035
        self.car_detected_period = 0.3
        self.car_detected = False
        self.init_period = 0.0

    def scan_callback(self, scan_data):

        ranges = scan_data.ranges
        angle_increment = scan_data.angle_increment
        secs = scan_data.header.stamp.sec
        nsecs = scan_data.header.stamp.nanosec

        found_start = False
        found_end = False

        delta = float("inf")
        line_length = 0.0

        # To keep track of how long another car is detected
        if self.car_detected:
            if self.init_period == 0.0:
                self.init_period = secs + nsecs * 1e-9
            curr_period = secs + nsecs * 1e-9
            self.car_detected_period = curr_period - self.init_period

        else:
            self.car_detected_period = 0.0
            self.init_period = 0.0

        for i in range(1, len(ranges)):

            delta = abs(ranges[i] - ranges[i - 1])

            # Finding start and end of car by detecting edges using a distance_threshold
            if delta > self.distance_threshold and found_start != True:
                found_start = True
                start_of_car = i

            elif (
                delta > self.distance_threshold
                and found_start == True
                and found_end != True
            ):
                found_end = True
                end_of_car = i - 1

            # Once edges found calculate the width of the range
            if found_start and found_end:
                for i in range(start_of_car, end_of_car):

                    try:
                        a = ranges[i + 1]
                        b = ranges[i]

                        if abs(a - b) < self.distance_threshold_car:
                            if (
                                0.25 < line_length < 0.35
                                and i == (end_of_car - 1)
                                and a < 20.0
                                and b < 20.0
                            ):

                                # Checking for false positives
                                if self.car_detected_period > 0.5:
                                    print(f"Car detected")

                                self.car_detected = True

                            line_length += math.sqrt(
                                a**2 + b**2 - 2 * a * b * math.cos(angle_increment)
                            )

                    except IndexError:
                        pass

                if self.car_detected == True and self.car_detected_period > 0.5:
                    self.car_detected = False
                found_end = False
                start_of_car = end_of_car
                line_length = 0.0


def main(args=None):

    rclpy.init(args=args)

    overtake = Overtake()

    rclpy.spin(overtake)

    overtake.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":

    main()

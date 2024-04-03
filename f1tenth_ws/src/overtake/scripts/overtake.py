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

    def scan_callback(self, scan_data):
        ranges = scan_data.ranges
        angle_increment = scan_data.angle_increment

        found_start = False
        found_end = False
        delta = float("inf")
        line_length = 0.0

        # THOUGHTS: When an end is detected it could also be the start of the car?
        # Loop through the ranges and find when there is a big jump in the distance
        for i in range(1, len(ranges)):

            delta = abs(ranges[i] - ranges[i - 1])

            # print(f"{i} | {ranges[i-1]:.2f} | {ranges[i]:.2f} |  {delta:.2f}")
            # print(f"{i} | {delta:.2f}")
            if delta > self.distance_threshold and found_start != True:
                found_start = True
                start_of_car = i
                # print(
                #     f"Start: {i} | {ranges[i-1]:.2f} | {ranges[i]:.2f} |  {delta:.2f}"
                # )

            elif (
                delta > self.distance_threshold
                and found_start == True
                and found_end != True
            ):
                found_end = True
                end_of_car = i - 1
                # print(f"End: {i} | {ranges[i-1]:.2f} | {ranges[i]:.2f} |  {delta:.2f}")

            if found_start and found_end:
                # print(f"Start: {start_of_car} | End: {end_of_car}")
                for i in range(start_of_car, end_of_car):
                    try:
                        a = ranges[i + 1]
                        b = ranges[i]
                        # print(f"a-b: {abs(a-b):.2f}")

                        if abs(a - b) < self.distance_threshold_car:

                            # print(
                            #     f"a: {a:.2f} | b: {b:.2f} | a-b: {abs(a-b):.2f} | line_length: {line_length:.2f}"
                            # )

                            if 0.25 < line_length < 0.35 and i == (end_of_car - 1):
                                if a < 20.0 and b < 20.0:

                                    print(f"Car detected")
                                    # print(
                                    #     f"a: {a:.2f} | b: {b:.2f} | a-b: {abs(a-b):.2f} | line_length: {line_length:.2f}"
                                    # )
                                    line_length = 0.0

                            # print(line_length)
                            line_length += math.sqrt(
                                a**2 + b**2 - 2 * a * b * math.cos(angle_increment)
                            )
                    except IndexError:
                        pass
                found_end = False
                found_start = True
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

#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

import numpy as np


class WallFollow(Node):

    def __init__(self):
        super().__init__("wall_follow")
        # self.declare_parameters(
        #     namespace="", parameters=[("lookahead_distance_gain", None)]
        # )

        # Subscribing to relevant topics
        self.sub_scan = self.create_subscription(
            LaserScan, "scan", self.scan_callback, 1
        )

        self.sub_odom = self.create_subscription(
            Odometry, "odom", self.odom_callback, 1
        )

        self.pub_drive = self.create_publisher(AckermannDriveStamped, "drive", 1)

        self.drive_msg = AckermannDriveStamped()

        # self.lookahead_distance_gain = self.get_parameter(
        #     "lookahead_distance_gain"
        # ).value

        self.Kp = 0.40
        self.Ki = 0.030
        self.Kd = 0.002

        self.integral = 0.0
        self.prev_error_1 = 0.0
        self.prev_secs = 0.0
        self.prev_nsecs = 0.0

        self.longitudinal_vel = 0
        self.coeffiecient_of_friction = 0.30
        self.wheel_base = 0.33

    def getRange(self, scan_data, angle):
        ranges = scan_data.ranges
        angle_rad = angle * (np.pi / 180)
        index = int(abs(angle_rad - scan_data.angle_max) / scan_data.angle_increment)

        return ranges[index]

    def odom_callback(self, odom_data):
        self.longitudinal_vel = odom_data.twist.twist.linear.x

    def scan_callback(self, scan_data):

        secs = scan_data.header.stamp.sec
        nsecs = scan_data.header.stamp.nanosec

        angle_b = 90
        angle_a = 40

        theta = (angle_b - angle_a) * (np.pi / 180)
        # 90 Degrees to the car
        distance_b = self.getRange(scan_data, angle_b)  # ranges[901]
        # ~ 35 Degrees to the first scan
        distance_a = (
            self.getRange(scan_data, angle_a)
        ) / 2  # ranges[760]

        alpha = -1 * np.arctan2(
            (distance_a * np.cos(theta) - distance_b), (distance_a * np.sin(theta))
        )

        actual_distance = distance_b * np.cos(alpha)
        desired_distance = 1.0  # Metres

        error = desired_distance - actual_distance
        lookahead_distance = self.longitudinal_vel * 0.15

        error_1 = error + lookahead_distance * np.sin(alpha)

        if (
            (self.prev_secs == 0.0)
            & (self.prev_nsecs == 0.0)
            & (self.prev_error_1 == 0.0)
        ):
            self.prev_secs = secs
            self.prev_nsecs = nsecs
            self.prev_error_1 = error_1

        dt = secs - self.prev_secs + (nsecs - self.prev_nsecs) * 1e-9

        try:
            self.integral += error_1 * dt

            steering_angle = (
                (self.Kp * error_1)
                + (self.Ki * self.integral)
                + (self.Kd * (error_1 - self.prev_error_1) / dt)
            )

            if steering_angle < -0.4:
                steering_angle = -0.4
            elif steering_angle > 0.4:
                steering_angle = 0.4
            self.drive_msg.drive.steering_angle = steering_angle

            # steering_angle = 0.3
            # self.drive_msg.drive.steering_angle = -0.3
            steering_angle_degrees = abs(steering_angle * (180 / np.pi))

            self.prev_error_1 = error_1
            self.prev_secs = secs
            self.prev_nsecs = nsecs

            # self.drive_msg.drive.speed = 0.8 * (1 / 1.2) ** (
            #     steering_angle_degrees - 15
            # )

            self.drive_msg.drive.speed = min(
                5.0,
                np.sqrt(
                    (10 * self.coeffiecient_of_friction * self.wheel_base)
                    / np.abs(np.tan(steering_angle))
                ),
            )

            self.pub_drive.publish(self.drive_msg)
            self.get_logger().info(
                f"steering_angle: {steering_angle_degrees:.2f} | speed: {self.longitudinal_vel:.2f}"
            )
        except ZeroDivisionError:
            pass


def main(args=None):
    rclpy.init(args=args)
    wall_follow = WallFollow()
    rclpy.spin(wall_follow)
    wall_follow.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":

    main()

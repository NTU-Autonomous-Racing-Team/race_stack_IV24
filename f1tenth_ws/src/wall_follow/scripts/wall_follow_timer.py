#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

import numpy as np
import math


class WallFollow(Node):

    def __init__(self):
        super().__init__("wall_follow")

        # Subscribing to relevant topics
        self.sub_scan = self.create_subscription(
            LaserScan, "scan", self.scan_callback, 1
        )

        self.sub_odom = self.create_subscription(
            Odometry, "odom", self.odom_callback, 1
        )

        self.pub_drive = self.create_publisher(AckermannDriveStamped, "drive", 1)

        timer_frequency = 38
        timer_period = 1.0 / timer_frequency
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # PID contants
        self.Kp = 0.20
        self.Ki = 0.010
        self.Kd = 0.002
        self.integral = 0.0

        # Variables to store the previous error and time
        self.prev_error_1 = 0.0
        self.prev_secs = 0.0
        self.prev_nsecs = 0.0
        self.secs = 0.0
        self.nsecs = 0.0

        self.longitudinal_vel = 0
        self.coeffiecient_of_friction = 1.2
        self.wheel_base = 0.33

        self.angle_b = 90
        self.angle_a = 40
        self.distance_a = 0.0
        self.distance_b = 0.0

        self.drive_msg = AckermannDriveStamped()

    # Function to get the range of the LIDAR scan data at a specific angle
    def getRange(self, scan_data, angle):
        ranges = scan_data.ranges
        angle_rad = angle * (np.pi / 180)
        index = int(abs(angle_rad - scan_data.angle_max) / scan_data.angle_increment)

        return ranges[index]

    def odom_callback(self, odom_data):
        self.longitudinal_vel = odom_data.twist.twist.linear.x

    def scan_callback(self, scan_data):

        self.secs = scan_data.header.stamp.sec
        self.nsecs = scan_data.header.stamp.nanosec

        # 90 Degrees to the car
        self.distance_b = self.getRange(scan_data, self.angle_b)  # ranges[901]
        # ~ 35 Degrees to the first scan
        self.distance_a = self.getRange(scan_data, self.angle_a)  # ranges[760]

    def timer_callback(self):

        theta = (self.angle_b - self.angle_a) * (np.pi / 180)

        alpha = -1 * np.arctan2(
            (self.distance_a * np.cos(theta) - self.distance_b),
            (self.distance_a * np.sin(theta)),
        )

        actual_distance = self.distance_b * np.cos(alpha)
        desired_distance = 1.2  # Metres

        error = desired_distance - actual_distance
        lookahead_distance = self.longitudinal_vel * 0.45  # Metres

        error_1 = error + lookahead_distance * np.sin(alpha)

        if (
            (self.prev_secs == 0.0)
            & (self.prev_nsecs == 0.0)
            & (self.prev_error_1 == 0.0)
        ):
            self.prev_secs = self.secs
            self.prev_nsecs = self.nsecs
            self.prev_error_1 = error_1

        dt = self.secs - self.prev_secs + (self.nsecs - self.prev_nsecs) * 1e-9

        if dt != 0.0:
            self.integral += error_1 * dt

            steering_angle = (
                (self.Kp * error_1)
                + (self.Ki * self.integral)
                + (self.Kd * (error_1 - self.prev_error_1) / dt)
            )
        else:
            steering_angle = 0.0

        # if math.isnan(steering_angle):
        #     steering_angle = 0.00

        self.prev_error_1 = error_1
        self.prev_secs = self.secs
        self.prev_nsecs = self.nsecs

        # Publishing the drive message
        self.drive_msg.drive.steering_angle = steering_angle
        self.drive_msg.drive.speed = max(
            2.0,
            np.sqrt(
                (10 * self.coeffiecient_of_friction * self.wheel_base)
                / np.abs(np.tan(steering_angle))
            ),
        )

        self.pub_drive.publish(self.drive_msg)

        self.get_logger().info(
            f"dt: {dt:.2f} | steering_angle: {steering_angle:.2f} | speed: {self.drive_msg.drive.speed:.2f}"
        )


def main(args=None):

    rclpy.init(args=args)

    wall_follow = WallFollow()

    rclpy.spin(wall_follow)

    wall_follow.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":

    main()

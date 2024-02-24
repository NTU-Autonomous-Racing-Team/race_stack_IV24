#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from typing import SupportsFloat, List, Tuple

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

from pid import PID

# reference: https://github.com/f1tenth/f1tenth_labs_openrepo/blob/main/f1tenth_lab4/README.md
# TODO-make unique topic name for mux


class GapFinderAlgorithm:
    def __init__(self, safety_bubble_diameter: float = 1.6, scan_angle_increment: float = 0.00435):
        self.safety_bubble_diameter = safety_bubble_diameter  # [m]
        self.scan_angle_increment = scan_angle_increment  # [rad]
        self.view_angle = 1.4  # [rad]
        self.speed_pid = PID(Kp=0.5, Ki=0.0, Kd=0.0)
        self.steering_pid = PID(Kp=0.5, Ki=0.0, Kd=0.0)

    def limit_search(self):
        left_bound = int((len(self.ranges)- self.view_angle//self.scan_angle_increment)/2)
        right_bound = int(left_bound + self.view_angle//self.scan_angle_increment)
        self.ranges = self.ranges[left_bound:right_bound]

    def find_min_range(self):
        self.min_range = min(self.ranges)
        self.min_range_index = int(self.ranges.index(self.min_range))
        # self.min_range_index = int(min(range(len(self.ranges)), key=self.ranges.__getitem__))

    def generate_safety_bubble(self):
        # set the ranges of the safety bubble to 0
        arc_increment = float(self.min_range * self.scan_angle_increment)
        radius_count = int(self.safety_bubble_diameter / 2 / arc_increment)
        for i in range(
            max(0, self.min_range_index - radius_count), min(self.min_range_index + radius_count + 1, len(self.ranges))
        ):
            self.ranges[i] = 0.0

    def find_max_gap(self):
        # if the closest point is on the left side of the car, then the max gap is on the right side of the car.
        # find the index of the max range in the ranges
        if self.min_range_index < len(self.ranges) // 2:
            self.max_gap_index = self.ranges.index(max(self.ranges[self.min_range_index :]))
        else:
            self.max_gap_index = self.ranges.index(max(self.ranges[: self.min_range_index]))

    def find_twist(self):
        turning_factor = 0.6
        speed_factor = 0.2
        # find the twist required to go to the max range in the max gap
        angZ = self.scan_angle_increment * (len(self.ranges) // 2 - self.max_gap_index)
        angZ = self.steering_pid.update(angZ, self.dt)
        # linear velocity is proportional to the max range
        linX = self.min_range
        linX = self.speed_pid.update(linX, self.dt)
        self.twist = [linX, angZ]

    def update(self, ranges, dt = 0.05):
        self.ranges = ranges
        self.dt = dt
        self.limit_search()
        self.find_min_range()
        self.generate_safety_bubble()
        self.find_max_gap()
        self.find_twist()
        return self.twist


class GapFinderNode(Node):
    def __init__(self, period=0.05):
        super().__init__("gap_finder")
        # Scan Subscriber
        self.scan_subscriber = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.scan_subscriber  # prevent unused variable warning
        # Odom Subscriber
        self.odom_subscriber = self.create_subscription(Odometry, "/ego_racecar/odom", self.odom_callback, 10)
        self.odom_subscriber
        # Drive Publisher
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.timer = self.create_timer(period , self.timer_callback)
        # GapFinder Algorithm
        self.gapFinderAlgorithm = GapFinderAlgorithm()
        # Memory
        self.period = period
        self.last_linX = 0.0
        self.last_angZ = 0.0
        self.ranges = []

    def scan_callback(self, scan_msg):
        # change in such a way that the first index is the most left range
        self.angle_min = scan_msg.angle_min
        self.angle_max = scan_msg.angle_max
        self.gapFinderAlgorithm.scan_angle_increment = scan_msg.angle_increment
        self.ranges = scan_msg.ranges

    def odom_callback(self, odom_msg):
        self.linX = odom_msg.twist.twist.linear.x
        self.angZ = odom_msg.twist.twist.angular.z

    def timer_callback(self):
        if len(self.ranges) != 0:
            self.run()

    def apply_filter(self):
        filter_factor = 0.7
        self.twist[0] = self.twist[0] * filter_factor + self.last_linX * (1 - filter_factor)
        self.twist[1] = self.twist[1] * filter_factor + self.last_angZ * (1 - filter_factor)
        self.last_linX = self.twist[0]
        self.last_angZ = self.twist[1]

    def publish_drive_msg(self):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.twist[0]
        drive_msg.drive.steering_angle = self.twist[1]
        self.drive_publisher.publish(drive_msg)

    def run(self):
        self.twist = self.gapFinderAlgorithm.update(self.ranges, self.period)
        # self.apply_filter()
        self.publish_drive_msg()


def main(args=None):
    rclpy.init(args=args)

    gapFinder = GapFinderNode()

    rclpy.spin(gapFinder)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gapFinder.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
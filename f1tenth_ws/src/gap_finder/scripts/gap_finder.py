#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

# reference: https://github.com/f1tenth/f1tenth_labs_openrepo/blob/main/f1tenth_lab4/README.md
# TODO-make unique topic name for mux


class GapFinderAlgorithm:
    def __init__(self, safety_bubble=0.4, angle_increment=0.00435):
        self.safety_bubble = safety_bubble  # [m]
        self.angle_increment = angle_increment  # [rad]

    def find_min_range(self):
        self.min_range = min(self.ranges)
        self.index_min = min(range(len(self.ranges)), key=self.ranges.__getitem__)

    def generate_safety_bubble(self):
        increment_arc = self.min_range * self.angle_increment
        radius_count = self.safety_bubble / arc_thetha // 2
        for i in range(
            int(self.index_min - radius_count), int(self.index_min + radius_count + 1)
        ):
            self.ranges[i] = 0.0

    def find_max_gap(self):
        # if the closest point is on the left side of the car, then the max gap is on the right side of the car.
        # find the index of the max range in the ranges
        if self.index_min < len(self.range) // 2:
            self.max_gap_index = self.ranges.index(max(self.ranges[self.index_min :]))
        else:
            self.max_gap_index = self.ranges.index(max(self.ranges[: self.index_min]))

    def find_twist(self):
        # find the twist required to go to the max range in the max gap
        angZ = self.angle_increment * (len(self.range) // 2 - self.max_gap_index)
        # linear velocity is proportional to the max range
        linX = self.min_range
        self.twist = [linX, angZ]

    def update(self, ranges):
        self.ranges = ranges
        self.find_min_range()
        self.generate_safety_bubble()
        self.find_max_gap()
        self.find_twist()
        return self.twist


class GapFinderNode(Node):
    def __init__(self, pub_rate=20):
        super().__init__("gap_finder")
        # Scan Subscriber
        self.scan_subscriber = self.create_subscription(LaserScan, "/gap_finder_scan", self.scan_callback, 10)
        self.scan_subscriber  # prevent unused variable warning
        # Odom Subscriber
        self.odom_subscriber = self.create_subscription(Odometry, "gap_finder/odom", self.odom_callback, 10)
        self.odom_subscriber
        # Drive Publisher
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, "/aeb/drive", 10)
        self.timer = self.create_timer(1 / pub_rate, self.timer_callback)
        # GapFinder Algorithm
        self.gapFinderAlgorithm = GapFinderAlgorithm()
        # Memory
        self.last_linX = 0.0
        self.last_angZ = 0.0

    def scan_callback(self, scan_msg):
        # change in such a way that the first index is the most left range
        self.ranges = scanConfig.reorientate(scan_msg.ranges)

    def odom_callback(self, odom_msg):
        self.linX = odom_msg.twist.twist.linear.x
        self.angZ = odom_msg.twist.twist.angular.z

    def timer_callback(self):
        self.run()

    def apply_filter(self):
        self.twist[0] = self.twist[0] * 0.5 + self.last_linX * 0.5
        self.twist[1] = self.twist[1] * 0.5 + self.last_angZ * 0.5
        self.last_linX = self.twist[0]

    def publish_drive_msg(self):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.twist[0]
        drive_msg.drive.steering_angle = self.twist[1]
        self.drive_publisher.publish(drive_msg)

    def run(self):
        self.twist = gapFinderAlgorithm.update(self.ranges)
        self.apply_filter()
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
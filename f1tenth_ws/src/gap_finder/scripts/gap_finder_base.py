#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

import numpy as np

from gap_finder.pid import PID

# reference: https://github.com/f1tenth/f1tenth_labs_openrepo/blob/main/f1tenth_lab4/README.md

class GapFinderAlgorithm:
    """
    This class implements the gap finder algorithm. The algorithm takes in a list of ranges from a LiDAR scan and
    returns a twist message that will move the car to the deepest gap in the scan.
    params:
        - safety_bubble_diameter: the diameter of the safety bubble to draw around obstacles
        - view_angle: the angle of the field of view of the LiDAR as a cone in front of the car
        - speed_pid: a PID controller for the linear velocity
        - steering_pid: a PID controller for the angular velocity
    """
    def __init__(self, safety_bubble_diameter = 0.6, view_angle = 1.4, coeffiecient_of_friction = 0.8):
        # Tunable Parameters
        self.safety_bubble_diameter = safety_bubble_diameter  # [m]
        self.view_angle = view_angle  # [rad]
        self.coeffiecient_of_friction = coeffiecient_of_friction
        self.wheel_base = 0.324  # [m]
        self.max_steering = 0.4  # [rad]
        # Controller Parameters
        self.speed_pid = PID(Kp=-1., Ki=0.0, Kd=0.0)
        self.speed_pid.set_point = 0.0
        self.steering_pid = PID(Kp=-1, Ki=0.0, Kd=0.0)
        self.steering_pid.set_point = 0.0

    def update(self, ranges, angle_increment, dt):
        ranges = np.array(ranges)
        ### LIMIT FIELD OF VIEW ###
        view_angle_count = self.view_angle//angle_increment
        lower_bound = int((ranges.shape[0] - view_angle_count)/2)
        upper_bound = int(lower_bound + view_angle_count)
        ranges = ranges[lower_bound:upper_bound]

        ### DRAW SAFETY BUBBLE ###
        min_range = np.min(ranges)
        min_range_index = np.argmin(ranges)
        arc_increment = float(min_range * angle_increment)
        radius_count = int(self.safety_bubble_diameter/2 / arc_increment)
        ranges[min_range_index - radius_count : min_range_index + radius_count + 1] = 0.0

        ### FIND MAX AVERAGE GAP ###
        if min_range_index < ranges.shape[0] // 2:
            # min is on right, turn left
            ranges = ranges[min_range_index :]
        else:
            # min is on left, turn right
            ranges = ranges[: min_range_index]

        half_window_size_array = (np.power(ranges * angle_increment, -1) * self.safety_bubble_diameter / 2).astype(int)
        for i, half_window_size in enumerate(half_window_size_array):
            if i < half_window_size:
                ranges[i] = np.mean(ranges[:i + half_window_size])
            elif i > ranges.shape[0] - half_window_size:
                ranges[i] = np.mean(ranges[i - half_window_size:])
            else:
                ranges[i] = np.mean(ranges[i - half_window_size: i + half_window_size])

        max_gap_index = np.argmax(ranges)

        ### FIND TWIST ###
        # find the twist required to go to the max range in the max gap
        init_steering = angle_increment * (max_gap_index - ranges.shape[0] // 2)
        steering = self.steering_pid.update(init_steering, dt)
        steering = np.sign(steering) * min(np.abs(steering, self.max_steering))
        # linear velocity uses the maximum linear speed that can be achieved with the current steering angle given the coefficient of friction
        init_speed = np.sqrt(10 * self.coeffiecient_of_friction * self.wheel_base / np.abs(np.tan(steering)))
        speed = self.speed_pid.update(init_speed, dt)
        ackermann = [speed, steering]
        return ackermann


class GapFinderNode(Node):
    """
    ROS2 Node Class that handles all the subscibers and publishers for the gap finder algorithm. 
    It abstracts the gap finder algorithm from the ROS2 interface.
    """
    def __init__(self, hz=50):
        super().__init__("gap_finder")
        # Timeouts
        self.timeout = 1.0 # [s]
        # Speed limits
        self.max_speed = 10.0 # [m/s]
        self.min_speed = 1.0 # [m/s]
        # Steering limits
        self.max_steering = 0.4 # [rad]
        # Scan Subscriber
        self.scan_subscriber = self.create_subscription(LaserScan, "scan", self.scan_callback, 1)
        self.scan_subscriber  # prevent unused variable warning
        self.scan_ready = False
        self.ranges = []
        self.scan_angle_increment = 0.0
        self.last_scan_time = self.get_time()
        # Odom Subscriber
        self.odom_subscriber = self.create_subscription(Odometry, "odom", self.odom_callback, 1)
        self.odom_subscriber
        self.odom_ready = False
        self.last_odom_time = self.get_time()
        # Drive Publisher
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, "nav/drive", 1)
        self.timer = self.create_timer(1/hz , self.timer_callback)
        # GapFinder Algorithm
        self.gapFinderAlgorithm = GapFinderAlgorithm()
        # Memory
        self.last_time = self.get_time()

    def get_time(self):
        return self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9

    def scan_callback(self, scan_msg):
        self.scan_ready = True
        self.angle_min = scan_msg.angle_min
        self.angle_max = scan_msg.angle_max
        self.scan_angle_increment = scan_msg.angle_increment
        self.ranges = scan_msg.ranges
        self.last_scan_time = self.get_time()

    def odom_callback(self, odom_msg):
        self.odom_ready = True
        self.linX = odom_msg.twist.twist.linear.x
        self.angZ = odom_msg.twist.twist.angular.z
        self.last_odom_time = self.get_time()

    def publish_drive_msg(self, twist=[0.0, 0.0]):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = float(twist[0])
        drive_msg.drive.steering_angle = float(twist[1])
        self.drive_publisher.publish(drive_msg)

    def timer_callback(self):
        if self.scan_ready and self.odom_ready:
            dt = self.get_time() - self.last_time

            twist = self.gapFinderAlgorithm.update(self.ranges, self.scan_angle_increment, dt)

            # steering limits
            twist[1] = np.sign(twist[1]) * min(np.abs(twist[1]), self.max_steering)
            # speed limits
            twist[0] = max(twist[0], self.min_speed)
            twist[0] = min(twist[0], self.max_speed)

            self.publish_drive_msg(twist)
            self.last_time = self.get_time()

        if ((self.get_time() - self.last_scan_time) > self.timeout):
            self.scan_ready = False
        if ((self.get_time() - self.last_odom_time) > self.timeout):
            self.odom_ready = False

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

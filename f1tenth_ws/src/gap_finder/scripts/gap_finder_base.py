#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

import numpy as np

# from gap_finder.pid import PID
from pid import PID

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
    def __init__(self, safety_bubble_diameter = 0.6, view_angle = 3.142/4*2, coeffiecient_of_friction = 0.8):
        # Tunable Parameters
        self.safety_bubble_diameter = safety_bubble_diameter  # [m]
        self.view_angle = view_angle  # [rad]
        self.coeffiecient_of_friction = coeffiecient_of_friction
        self.wheel_base = 0.324  # [m]
        self.max_steering = 0.4  # [rad]
        # Controller Parameters
        self.speed_pid = PID(Kp=-0.5, Ki=0.0, Kd=0.0)
        self.speed_pid.set_point = 0.0
        self.steering_pid = PID(Kp=-1.2, Ki=0.0, Kd=0.005)
        self.steering_pid.set_point = 0.0

    def update(self, ranges, angle_increment, dt):
        ranges = np.array(ranges)
        ranges[ranges > 30] = 30
        ### LIMIT FIELD OF VIEW ###
        view_angle_count = self.view_angle//angle_increment
        lower_bound = int((ranges.shape[0] - view_angle_count)/2)
        upper_bound = int(lower_bound + view_angle_count)
        ranges = ranges[lower_bound:upper_bound]

        ### SPLIT SCAN INTO LEFT AND RIGHT ###
        ranges_left = ranges[ranges.shape[0]//2:]
        ranges_right = ranges[:ranges.shape[0]//2]

        ### DRAW SAFETY BUBBLES ###
        # LEFT SAFETY BUBBLE
        min_range = np.min(ranges_left)
        min_range_index = np.argmin(ranges_left)
        arc_increment = float(min_range * angle_increment)
        radius_count = int(self.safety_bubble_diameter/2 / np.nan_to_num(arc_increment))
        ranges_left[min_range_index - radius_count : min_range_index + radius_count + 1] = 9999
        # for visialisation
        self.min_range_bearing_1 = angle_increment * (min_range_index - ranges_left.shape[0] // 2)
        self.min_range_1 = min_range

        # RIGHT SAFETY BUBBLE
        min_range = np.min(ranges_right)
        min_range_index = np.argmin(ranges_right)
        arc_increment = float(min_range * angle_increment)
        radius_count = int(self.safety_bubble_diameter/2 / np.nan_to_num(arc_increment))
        ranges_right[min_range_index - radius_count : min_range_index + radius_count + 1] = 9999
        # for visialisation
        self.min_range_bearing_2 = angle_increment * (min_range_index - ranges.shape[0] // 2)
        self.min_range_2 = min_range

        # combine left and right
        ranges = np.concatenate((ranges_right, ranges_left))
        ranges[ranges == 9999] = 0.0

        ### APPLY MEAN FILTER ###
        arc_increments = ranges * angle_increment
        half_window_size_array = (self.safety_bubble_diameter/2 / arc_increments).astype(int)
        half_window_size_array[half_window_size_array == 0] = 1
        # half_window_size_array = (np.power(ranges * angle_increment, -1) * self.safety_bubble_diameter / 2).astype(int)
        for i, half_window_size in enumerate(half_window_size_array):
            if ranges[i] <= 1e-9:
                # within the safety bubble
                continue
            elif i < half_window_size:
                ranges[i] = np.mean(ranges[:i + half_window_size])
            elif i > ranges.shape[0] - half_window_size:
                ranges[i] = np.mean(ranges[i - half_window_size:])
            else:
                ranges[i] = np.mean(ranges[i - half_window_size: i + half_window_size])

        ### PRIORITISE CENTER OF SCAN ###
        mask_right = np.linspace(0.99, 1, ranges.shape[0]//2)
        mask_left = np.linspace(1, 0.99, ranges.shape[0]//2)
        mask = np.concatenate((mask_right, mask_left))
        if mask.shape[0] < ranges.shape[0]:
            mask = np.concatenate((mask, [mask_right[-1]]))
        ranges *= mask
        np.nan_to_num(ranges)

        ### FIND MAX AVERAGE GAP ###
        # if min_range_index < ranges.shape[0] // 2:
        #     # min is on right, turn left
        #     ranges = ranges[min_range_index :]
        # else:
        #     # min is on left, turn right
        #     ranges = ranges[: min_range_index]

        max_gap_index = np.argmax(ranges)
        self.max_range = np.max(ranges)
        # print(f"index: {max_gap_index}, range:{ranges[max_gap_index]}")

        ### FIND TWIST ###
        # find the twist required to go to the max range in the max gap
        init_steering = angle_increment * (max_gap_index - ranges.shape[0] // 2)
        self.max_angle = init_steering
        steering = self.steering_pid.update(init_steering, dt)
        steering = np.sign(steering) * min(np.abs(steering), self.max_steering)
        # linear velocity uses the maximum linear speed that can be achieved with the current steering angle given the coefficient of friction
        init_speed = np.sqrt(10 * self.coeffiecient_of_friction * self.wheel_base / np.abs(max(np.tan(steering),1e-9)))
        speed = self.speed_pid.update(init_speed, dt)
        ackermann = [speed, steering]
        return ackermann
    
    def get_safety_1_coord(self):
        x = self.min_range_1 * np.cos(self.min_angle_1)
        y = self.min_range_1 * np.sin(self.min_angle_1)
        return [x, y]

    def get_safety_2_coord(self):
        x = self.min_range_2 * np.cos(self.min_angle_2)
        y = self.min_range_2 * np.sin(self.min_angle_2)
        return [x, y]

    def get_goal_coord(self):
        x = self.max_range * np.cos(self.max_angle)
        y = self.max_range * np.sin(self.max_angle)
        return [x, y]


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
        self.odom_subscriber = self.create_subscription(Odometry, "ego_racecar/odom", self.odom_callback, 1)
        self.odom_subscriber
        self.odom_ready = False
        self.last_odom_time = self.get_time()
        # Drive Publisher
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, "/drive", 1)
        self.timer = self.create_timer(1/hz , self.timer_callback)
        # Safety Viz Publisher
        self.bubble_1_viz_publisher = self.create_publisher(Marker, "/bubble_1", 1)
        self.bubble_2_viz_publisher = self.create_publisher(Marker, "/bubble_2", 1)
        # Goal Viz Publisher
        self.gap_viz_publisher = self.create_publisher(Marker, "/gap", 1)
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

    def publish_viz_msgs(self):
        bubble_1_coord = self.gapFinderAlgorithm.get_safety_1_coord()
        bubble_2_coord = self.gapFinderAlgorithm.get_safety_2_coord()
        goal_coord = self.gapFinderAlgorithm.get_goal_coord()
        bubble_1_viz_msg = Marker()
        bubble_2_viz_msg = Marker()
        gap_viz_msg = Marker()

        bubble_1_viz_msg.header.frame_id = "ego_racecar/base_link"
        bubble_1_viz_msg.pose.position.x = bubble_1_coord[0]
        bubble_1_viz_msg.pose.position.y = bubble_1_coord[1]
        bubble_1_viz_msg.color.a = 1.0
        bubble_1_viz_msg.color.r = 1.0
        bubble_1_viz_msg.scale.x = .6
        bubble_1_viz_msg.scale.y = .6
        bubble_1_viz_msg.scale.z = .6
        bubble_1_viz_msg.type = Marker().SPHERE
        bubble_1_viz_msg.action = Marker().ADD

        bubble_2_viz_msg.header.frame_id = "ego_racecar/base_link"
        bubble_2_viz_msg.pose.position.x = bubble_2_coord[0]
        bubble_2_viz_msg.pose.position.y = bubble_2_coord[1]
        bubble_2_viz_msg.color.a = 1.0
        bubble_2_viz_msg.color.r = 1.0
        bubble_2_viz_msg.scale.x = .6
        bubble_2_viz_msg.scale.y = .6
        bubble_2_viz_msg.scale.z = .6
        bubble_2_viz_msg.type = Marker().SPHERE
        bubble_2_viz_msg.action = Marker().ADD

        gap_viz_msg.header.frame_id = "ego_racecar/base_link"
        gap_viz_msg.pose.position.x = goal_coord[0]
        gap_viz_msg.pose.position.y = goal_coord[1]
        gap_viz_msg.color.a = 1.0
        gap_viz_msg.color.g = 1.0
        gap_viz_msg.scale.x = 1.0
        gap_viz_msg.scale.y = 1.0
        gap_viz_msg.scale.z = 1.0
        gap_viz_msg.type = Marker.SPHERE
        gap_viz_msg.action = Marker.ADD

        self.bubble_1_viz_publisher.publish(bubble_1_viz_msg)
        self.bubble_2_viz_publisher.publish(bubble_2_viz_msg)
        self.gap_viz_publisher.publish(gap_viz_msg)


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
        
            self.publish_viz_msgs()

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

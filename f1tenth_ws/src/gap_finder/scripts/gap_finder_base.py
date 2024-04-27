#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np

from gap_finder.pid import PID
# from pid import PID

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
    def __init__(self,  safety_bubble_diameter = 0.4, 
                        view_angle = 3.142, 
                        coeffiecient_of_friction = 0.71, 
                        vertice_detection_threshold = 0.6,
                        lookahead = 5.0, 
                        max_speed = 10.0, 
                        max_steering = 0.4):
        # Tunable Parameters
        self.safety_bubble_diameter = safety_bubble_diameter  # [m]
        self.view_angle = view_angle  # [rad]
        self.coeffiecient_of_friction = coeffiecient_of_friction
        self.lookahead = lookahead
        self.max_speed = max_speed
        self.max_steering = max_steering
        self.change_threshold = vertice_detection_threshold
        self.wheel_base = 0.324  # [m]
        # Controller Parameters
        self.speed_pid = PID(Kp=-1)
        self.speed_pid.set_point = 0.0
        self.steering_pid = PID(Kp=-1.2)
        self.steering_pid.set_point = 0.0
        # Filtering
        self.last_goal_bearing = 0.0
        self.filter_alpha = 1.0 # closer to 1 means no filtering

    def update(self, ranges, angle_increment, scan_msg):
        ranges = np.array(ranges)

        ### FIND FRONT CLEARANCE ###
        mid_index = ranges.shape[0]//2
        # mid_arc = ranges[mid_index] * angle_increment
        # front_clearance_count = int(self.safety_bubble_diameter/2 / mid_arc)
        # front_clearance = np.mean(ranges[(mid_index - front_clearance_count):(mid_index + front_clearance_count)])
        
        ### LIMIT LOOKAHEAD ##
        if self.lookahead is not None:
            ranges[ranges > self.lookahead] = self.lookahead
        cp_ranges = ranges.copy()

        ### MARK LARGE DERIVATIVE CHANGES###
        marked_indexes = []
        for i in range(1, ranges.shape[0]):
            if abs(ranges[i] - ranges[i-1]) > self.change_threshold:
                if ranges[i] < ranges[i-1]:
                    marked_indexes.append(i)
                else:
                    marked_indexes.append(i-1)

        ### MARK MINIMUM ON LEFT AND RIGHT ###
        # split ranges into left and right
        ranges_right = ranges[:ranges.shape[0]//2]
        ranges_left = ranges[ranges.shape[0]//2:]
        # find minimums on left and right
        min_range_index = np.argmin(ranges_left)
        marked_indexes.append(ranges.shape[0]//2 + min_range_index)
        min_range_index = np.argmin(ranges_right)
        marked_indexes.append(min_range_index)
        # recombine left and right
        ranges = np.concatenate((ranges_right, ranges_left))

        ### APPLY SAFETY BUBBLE ###
        self.marked_ranges = [] # for visualisation
        for i in marked_indexes:
            if cp_ranges[i] == 0.0:
                continue

            arc = angle_increment * cp_ranges[i]
            arc_increment = int(self.safety_bubble_diameter/arc/2)
            ranges[i-arc_increment:i+arc_increment+1] = 0.0

            # for visualisation
            bearing = angle_increment * (i - ranges.shape[0]//2)
            marker = [bearing, cp_ranges[i]]
            self.marked_ranges.append(marker)

        # for visualisation
        self.safety_scan_msg = scan_msg
        scan_msg.ranges = ranges.tolist()

        ### PRIORITISE CENTER OF SCAN ###
        mask_left = np.linspace(1.0, 0.99, ranges_left.shape[0])
        mask_right = np.linspace(0.99, 1.0, ranges_right.shape[0])
        mask = np.concatenate((mask_right, mask_left))
        ranges *= mask

        ### LIMIT FIELD OF VIEW ###
        view_angle_count = self.view_angle//angle_increment
        lower_bound = int((ranges.shape[0] - view_angle_count)/2)
        upper_bound = int(lower_bound + view_angle_count)

        ### FIND MAX AVERAGE GAP ###
        limited_range = ranges[lower_bound:upper_bound+1]
        max_gap_index = np.argmax(limited_range)
        self.goal_bearing = angle_increment * (max_gap_index - limited_range.shape[0] // 2)
        # target_bearing = angle_increment * (max_gap_index - limited_range.shape[0] // 2)
        # self.goal_bearing = self.last_goal_bearing * (1.0-self.filter_alpha) + target_bearing * self.filter_alpha
        # self.last_goal_bearing = target_bearing
        # target_index = int(self.goal_bearing/angle_increment + limited_range.shape[0]/2)
        self.goal_range = limited_range[max_gap_index]
        # self.goal_range = limited_range[target_index]

        ### FIND TWIST ###
        # init_steering = self.goal_bearing
        init_steering = np.arctan(self.goal_bearing * self.wheel_base)
        steering = self.steering_pid.update(init_steering)
        # steering = min(abs(steering), self.max_steering) * np.sign(steering)
        init_speed = np.sqrt(10 * self.coeffiecient_of_friction * self.wheel_base / np.abs(max(np.tan(abs(steering)),1e-9)))
        # init_speed = front_clearance / (1.0 * cp_ranges[max_gap_index]) * self.max_speed * np.cos(2*steering)
        speed = self.speed_pid.update(init_speed)
        ackermann = {"speed": speed, "steering": steering}
        return ackermann
    
    def get_bubble_coord(self):
        m = []
        for marker in self.marked_ranges:
            x = marker[1] * np.cos(marker[0])
            y = marker[1] * np.sin(marker[0])
            m.append([x, y])
        return m

    def get_goal_coord(self):
        x = self.goal_range * np.cos(self.goal_bearing)
        y = self.goal_range * np.sin(self.goal_bearing)
        return [x, y]

    def get_safety_scan(self):
        return self.safety_scan_msg


class GapFinderNode(Node):
    """
    ROS2 Node Class that handles all the subscibers and publishers for the gap finder algorithm. 
    It abstracts the gap finder algorithm from the ROS2 interface.
    """
    def __init__(self, hz=50):
        super().__init__("gap_finder")
        # self.safety_bubble_diameter = 0.6
        # Timeouts
        self.timeout = 1.0 # [s]
        # Speed limits
        self.max_speed = 10.0 # [m/s]
        self.min_speed = 1.0 # [m/s]
        self.max_acceleration = 10.0 # [m s^-2]
        # Steering limits
        self.max_steering = 0.4 # [rad]
        self.max_d_steering = 0.6 # [rad/s]
        # Scan Subscriber
        self.scan_subscriber = self.create_subscription(LaserScan, "scan", self.scan_callback, 1)
        self.scan_subscriber  # prevent unused variable warning
        self.scan_ready = False
        # self.ranges = []
        # self.scan_angle_increment = 0.0
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
        self.bubble_viz_publisher = self.create_publisher(MarkerArray, "/safety_bubble", 1)
        # Goal Viz Publisher
        self.gap_viz_publisher = self.create_publisher(Marker, "/goal_point", 1)
        # Laser Viz Publisher
        self.laser_publisher = self.create_publisher(LaserScan, "/safety_scan", 1)
        # GapFinder Algorithm
        self.gapFinderAlgorithm = GapFinderAlgorithm(safety_bubble_diameter = 0.6, 
                                                     view_angle= 3.142,
                                                     coeffiecient_of_friction= 0.5,
                                                     vertice_detection_threshold = 0.6/2,
                                                     lookahead = None, 
                                                     max_speed = self.max_speed, 
                                                     max_steering = self.max_steering)
        # Memory
        self.last_time = self.get_time()
        self.last_twist = {"speed": 0.0, "steering":0.0}

    def get_time(self):
        return self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9

    def scan_callback(self, scan_msg):
        self.scan_ready = True
        self.scan_msg = scan_msg
        self.last_scan_time = self.get_time()
        # self.angle_min = scan_msg.angle_min
        # self.angle_max = scan_msg.angle_max
        # self.scan_angle_increment = scan_msg.angle_increment
        # self.ranges = scan_msg.ranges

    def odom_callback(self, odom_msg):
        self.odom_ready = True
        self.odom_msg = odom_msg
        # self.linX = odom_msg.twist.twist.linear.x
        # self.angZ = odom_msg.twist.twist.angular.z
        self.last_odom_time = self.get_time()

    def publish_drive_msg(self, twist={"speed": 0.0, "steering": 0.0}):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = float(twist["speed"])
        drive_msg.drive.steering_angle = float(twist["steering"])
        self.drive_publisher.publish(drive_msg)

    def publish_viz_msgs(self):
        safety_scan = self.gapFinderAlgorithm.get_safety_scan()
        bubble_coord = self.gapFinderAlgorithm.get_bubble_coord()
        goal_coord = self.gapFinderAlgorithm.get_goal_coord()
        safety_bubble_diameter = self.gapFinderAlgorithm.safety_bubble_diameter
        bubble_viz_msg = MarkerArray()
        gap_viz_msg = Marker()
        laser_viz_msg = safety_scan

        for i, coord in enumerate(bubble_coord):
            marker = Marker()
            marker.id = i
            marker.header.frame_id = "ego_racecar/base_link"
            marker.pose.position.x = coord[0]
            marker.pose.position.y = coord[1]
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.scale.x = safety_bubble_diameter
            marker.scale.y = safety_bubble_diameter
            marker.scale.z = safety_bubble_diameter
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            bubble_viz_msg.markers.append(marker)

        gap_viz_msg.header.frame_id = "ego_racecar/base_link"
        gap_viz_msg.pose.position.x = goal_coord[0]
        gap_viz_msg.pose.position.y = goal_coord[1]
        gap_viz_msg.color.a = 1.0
        gap_viz_msg.color.g = 1.0
        gap_viz_msg.scale.x = 0.3
        gap_viz_msg.scale.y = 0.3
        gap_viz_msg.scale.z = 0.3
        gap_viz_msg.type = Marker.SPHERE
        gap_viz_msg.action = Marker.ADD

        self.bubble_viz_publisher.publish(bubble_viz_msg)
        self.gap_viz_publisher.publish(gap_viz_msg)
        self.laser_publisher.publish(laser_viz_msg)


    def timer_callback(self):
        if self.scan_ready and self.odom_ready:
            dt = self.get_time() - self.last_time

            twist = self.gapFinderAlgorithm.update(self.scan_msg.ranges, self.scan_msg.angle_increment, self.scan_msg)

            # steering limits
            twist["steering"] = np.sign(twist["steering"]) * min(np.abs(twist["steering"]), self.max_steering)
            # speed limits
            twist["speed"] = max(twist["speed"], self.min_speed)
            twist["speed"] = min(twist["speed"], self.max_speed)

            # control change limits
            # d_speed = twist["speed"] - self.last_twist["speed"]
            # if abs(d_speed) > self.max_acceleration*dt:
            #     twist["speed"] = self.last_twist["speed"] + np.sign(d_speed) * self.max_acceleration*dt

            # d_steering = twist["steering"] - self.last_twist["steering"]
            # if abs(d_steering) > self.max_d_steering*dt:
            #     twist["steering"] = self.last_twist["steering"] + np.sign(d_steering) * self.max_d_steering*dt
            
            # self.last_twist = twist
            # twist["speed"] = 0.0

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

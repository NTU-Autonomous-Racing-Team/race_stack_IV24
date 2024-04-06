#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

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
        - ADD SAFETY BUBBLE AT BIG DR CHANGE
    """
    def __init__(self,  safety_bubble_diameter = 0.6, 
                        view_angle = 4* 3.142/4, 
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
        # self.speed_pid = PID(Kp=-0.5, Ki=0.0, Kd=0.0)
        self.speed_pid = PID(Kp=-1, Ki=0.0, Kd=0.0)
        self.speed_pid.set_point = 0.0
        self.steering_pid = PID(Kp=-0.6, Ki=-0.0, Kd=0.0025)
        self.steering_pid.set_point = 0.0

    def draw_safety_bubble(self, index, angle_increment, ranges):
        arc_increment = float(ranges[index] * angle_increment)
        radius_count = int(self.safety_bubble_diameter/2 / arc_increment)
        ranges[index - radius_count : index + radius_count + 1] = 0.0
        return ranges

    def update(self, ranges, angle_increment, dt):
        ranges = np.array(ranges)
        
        ### LIMIT FIELD OF VIEW ###
        view_angle_count = self.view_angle//angle_increment
        lower_bound = int((ranges.shape[0] - view_angle_count)/2)
        upper_bound = int(lower_bound + view_angle_count)
        ranges = ranges[lower_bound:upper_bound+1]

        self.scan_angle_increment = angle_increment
        ranges[ranges > self.lookahead] = self.lookahead
        front_clearance = ranges[ranges.shape[0]//2]
        cp_ranges = np.copy(ranges)

        ### MARK LARGE DERIVATIVE CHANGES###
        for i in range(1, ranges.shape[0]):
            if ranges[i] - ranges[i-1] > self.change_threshold:
                ranges[i-1] = 9999
        ranges = np.flip(ranges)
        for i in range(1, ranges.shape[0]):
            if ranges[i] - ranges[i-1] > self.change_threshold:
                ranges[i-1] = 9999
        ranges = np.flip(ranges) 
 
        ### SPLIT SCAN INTO LEFT AND RIGHT ###
        ranges_right = ranges[:ranges.shape[0]//2]
        ranges_left = ranges[ranges.shape[0]//2:]

        ### MARK MINIMUM ON LEFT AND RIGHT ###
        ranges_left = np.flip(ranges_left)
        min_range_index = np.argmin(ranges_left)
        ranges_left[min_range_index] = 9999
        ranges_left = np.flip(ranges_left)

        # RIGHT SAFETY BUBBLE
        min_range_index = np.argmin(ranges_right)
        ranges_right[min_range_index] = 9999
      
        # COMBINE LEFT AND RIGHT
        ranges = np.concatenate((ranges_right, ranges_left))

        ### APPLY SAFETY BUBBLE ###
        self.marked_ranges = []
        for i, r in enumerate(ranges):
            marker = []
            if r == 9999:
                bearing = angle_increment * (i - ranges.shape[0]//2)
                marker.append(bearing)
                marker.append(cp_ranges[i])
                self.marked_ranges.append(marker)
                ranges = self.draw_safety_bubble(i, angle_increment, ranges)

        ranges[ranges == 9999] = 0.0

        ### APPLY MEAN FILTER ###
        arc_increments = ranges * angle_increment
        half_window_size_array = (self.safety_bubble_diameter/2 / arc_increments).astype(int)
        half_window_size_array[half_window_size_array == 0] = 1
        for i, half_window_size in enumerate(half_window_size_array):
            if ranges[i] <= 1e-9:
                continue
            elif i < half_window_size:
                ranges[i] = np.mean(ranges[:i + half_window_size])
            elif i > ranges.shape[0] - half_window_size:
                ranges[i] = np.mean(ranges[i - half_window_size:])
            else:
                ranges[i] = np.mean(ranges[i - half_window_size: i + half_window_size])

  
        ### PRIORITISE CENTER OF SCAN ###
        mask_left = np.linspace(1.0, 0.99, ranges_left.shape[0])
        mask_right = np.linspace(0.99, 1.0, ranges_right.shape[0])
        mask = np.concatenate((mask_right, mask_left))
        ranges *= mask

        ### FIND MAX AVERAGE GAP ###
        max_gap_index = np.argmax(ranges)
        self.goal_range = np.max(ranges)
        self.goal_bearing = angle_increment * (max_gap_index - ranges.shape[0] // 2)

        ### FIND TWIST ###
        init_steering = self.goal_bearing
        steering = self.steering_pid.update(init_steering, dt)
        steering = min(abs(steering), self.max_steering) * np.sign(steering)
        # init_speed = np.sqrt(10 * self.coeffiecient_of_friction * self.wheel_base / np.abs(max(np.tan(abs(steering)),1e-9)))
        init_speed = front_clearance / (1.0 * cp_ranges[max_gap_index]) * self.max_speed * np.cos(2*steering)
        speed = self.speed_pid.update(init_speed, dt)
        ackermann = [speed, steering]
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


class GapFinderNode(Node):
    """
    ROS2 Node Class that handles all the subscibers and publishers for the gap finder algorithm. 
    It abstracts the gap finder algorithm from the ROS2 interface.
    """
    def __init__(self, hz=50):
        super().__init__("gap_finder")
        self.safety_bubble_diameter = 1.3
        self.lookahead = 30.0
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
        self.bubble_viz_publisher = self.create_publisher(MarkerArray, "/bubble", 1)
        # Goal Viz Publisher
        self.gap_viz_publisher = self.create_publisher(Marker, "/gap", 1)
        # GapFinder Algorithm
        self.gapFinderAlgorithm = GapFinderAlgorithm(safety_bubble_diameter = self.safety_bubble_diameter, 
                                                     view_angle= 4* 3.142/4,
                                                     coeffiecient_of_friction= 0.5,
                                                     vertice_detection_threshold=self.safety_bubble_diameter/2,
                                                     lookahead = self.lookahead, 
                                                     max_speed = self.max_speed, 
                                                     max_steering = self.max_steering)
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
        bubble_coord = self.gapFinderAlgorithm.get_bubble_coord()
        goal_coord = self.gapFinderAlgorithm.get_goal_coord()
        bubble_viz_msg = MarkerArray()
        gap_viz_msg = Marker()

        for i, coord in enumerate(bubble_coord):
            marker = Marker()
            marker.header.frame_id = "ego_racecar/base_link"
            marker.pose.position.x = coord[0]
            marker.pose.position.y = coord[1]
            marker.id = i
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.scale.x = self.safety_bubble_diameter
            marker.scale.y = self.safety_bubble_diameter
            marker.scale.z = self.safety_bubble_diameter
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            bubble_viz_msg.markers.append(marker)

        gap_viz_msg.header.frame_id = "ego_racecar/base_link"
        gap_viz_msg.pose.position.x = goal_coord[0]
        gap_viz_msg.pose.position.y = goal_coord[1]
        # gap_viz_msg.pose.position.x = 0.0
        # gap_viz_msg.pose.position.y = 0.0
        gap_viz_msg.color.a = 1.0
        gap_viz_msg.color.g = 1.0
        gap_viz_msg.scale.x = 0.3
        gap_viz_msg.scale.y = 0.3
        gap_viz_msg.scale.z = 0.3
        gap_viz_msg.type = Marker.SPHERE
        gap_viz_msg.action = Marker.ADD

        self.bubble_viz_publisher.publish(bubble_viz_msg)
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

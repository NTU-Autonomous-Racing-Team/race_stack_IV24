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
# reference: https://www.nathanotterness.com/2019/04/the-disparity-extender-algorithm-and.html 

class GapFinderAlgorithm:
    """
    This class implements the gap finder algorithm. The algorithm takes in a list of ranges from a LiDAR scan and
    returns a twist dictionary that will move the car to the deepest gap in the scan after drawing safety bubbles.
    params:
        - safety_bubble_diameter: the diameter of the safety bubble
        - view_angle: the angle of the field of view of the LiDAR as a cone in front of the car
        - coeffiecient_of_friction: the coeffiecient of friction of the car used for speed calculation
        - disparity_threshold: the threshold for marking a disparity in the scan i.e. an edge
        - lookahead : the maximum distance to look ahead in the scan i.e ranges more than this are set to this value
        - speed_kp: the proportional gain for the speed PID controller
        - steering_kp: the proportional gain for the steering PID controller
        - wheel_base: the distance between the front and rear axles of the car
        - speed_max: the maximum speed of the car
        - visualise: a boolean to generate visualisation markers
    """
    def __init__(self,  safety_bubble_diameter = 0.4, 
                        view_angle = 3.142, 
                        coeffiecient_of_friction = 0.71, 
                        disparity_threshold = 0.6,
                        lookahead = None, 
                        speed_kp = 1.0,
                        steering_kp = 1.2,
                        wheel_base = 0.324, 
                        speed_max = 10.0,
                        visualise = False):
        # Tunable Parameters
        self.safety_bubble_diameter = safety_bubble_diameter  # [m]
        self.view_angle = view_angle  # [rad]
        self.coeffiecient_of_friction = coeffiecient_of_friction
        self.lookahead = lookahead # [m]
        self.disparity_threshold = disparity_threshold # [m]
        self.wheel_base = wheel_base  # [m]
        self.speed_max = speed_max  # [m/s]
        # Controller Parameters
        self.speed_pid = PID(Kp=-speed_kp)
        self.speed_pid.set_point = 0.0
        self.steering_pid = PID(Kp=-steering_kp)
        self.steering_pid.set_point = 0.0
        # Feature Activation
        self.do_mark_sides = True
        self.do_mean_filter = True
        self.do_limit_lookahead = True
        # Visualisation
        self.visualise = visualise
        self.safety_markers = {"range":[0.0], "bearing":[0.0]}
        self.goal_marker = {"range":0.0, "bearing":0.0}
        # Internal Variables
        self.initialise = True
        self.center_priority_mask = None
        self.fov_bounds = None
        self.middle_index = None

    def update(self, scan_msg):
        ranges = np.array(scan_msg.ranges)
        angle_increment = scan_msg.angle_increment

        if self.initialise:
            # lookahead
            if self.lookahead is None:
                self.lookahead = scan_msg.range_max
            # middle index for front of car
            self.middle_index = ranges.shape[0]//2
            # field of view bounds
            view_angle_count = self.view_angle//angle_increment
            lower_bound = int((ranges.shape[0] - view_angle_count)/2)
            upper_bound = int(lower_bound + view_angle_count)
            self.fov_bounds = [lower_bound, upper_bound+1]
            # center priority mask
            ranges_right = ranges[lower_bound:self.middle_index]
            ranges_left = ranges[self.middle_index:upper_bound+1]
            mask_right = np.linspace(0.999, 1.0, ranges_right.shape[0])
            mask_left = np.linspace(1.0, 0.999, ranges_left.shape[0])
            self.center_priority_mask = np.concatenate((mask_right, mask_left))
            self.initialise = False
 
        ### LIMIT LOOKAHEAD ##
        if (self.do_limit_lookahead):
            ranges[ranges > self.lookahead] = self.lookahead
            modified_ranges = ranges.copy()
        else :
            modified_ranges = ranges.copy()
            self.lookahead = scan_msg.range_max

        ### FIND FRONT CLEARANCE ###
        front_clearance = ranges[self.middle_index] # single laser scan
        if front_clearance != 0.0: # mean of safety bubble of front scan
            arc = angle_increment * ranges[self.middle_index]
            radius_count = int(self.safety_bubble_diameter/arc/2)
            front_clearance = np.mean(ranges[self.middle_index-radius_count:self.middle_index+radius_count])

        ### FIND MEAN RANGE ###
        mean_range = np.mean(ranges)

        ### MARK LARGE DISPARITY###
        marked_indexes = []
        for i in range(1, ranges.shape[0]):
            if abs(ranges[i] - ranges[i-1]) > self.disparity_threshold:
                if ranges[i] < ranges[i-1]:
                    marked_indexes.append([i, ranges[i]])
                else:
                    marked_indexes.append([i-1, ranges[i-1]])

        ### MARK MINIMUM ###
        marked_indexes.append([np.argmin(ranges), np.min(ranges)])

        ### MARK LEFT AND RIGHT ###
        if (self.do_mark_sides):
            marked_indexes.append([0, ranges[0]]) # right most
            marked_indexes.append([ranges.shape[0]-1, ranges[ranges.shape[0]-1]]) # left most

        ### MARK MINIMUM ON LEFT AND RIGHT ###
        # # split ranges into left and right
        # ranges_right = ranges[:mid_index]
        # ranges_left = ranges[mid_index:]
        # # find minimums on left and right
        # min_range_index = np.argmin(ranges_left)
        # marked_indexes.append(ranges.shape[0]//2 + min_range_index)
        # min_range_index = np.argmin(ranges_right)
        # marked_indexes.append(min_range_index)
        # # recombine left and right
        # ranges = np.concatenate((ranges_right, ranges_left))

        ### APPLY SAFETY BUBBLE ###
        # for i_range in marked_indexes:
        #     if i_range[1] == 0.0:
        #         continue
        #     arc = angle_increment * i_range[1]
        #     radius_count = int(self.safety_bubble_diameter/arc/2)
        #     modified_ranges[i_range[0]-radius_count:i_range[0]+radius_count+1] = i_range[1]

        ### LIMIT FIELD OF VIEW ###
        limited_ranges = modified_ranges[self.fov_bounds[0]:self.fov_bounds[1]]
        temp_ranges = limited_ranges.copy()

        ### MEAN FILTER ###
        if (self.do_mean_filter):
            for i, r in enumerate(temp_ranges):
                arc = angle_increment * r
                radius_count = int(self.safety_bubble_diameter/arc/2)
                if i < radius_count:
                    the_min = np.min(temp_ranges[:i+radius_count+1])
                elif i > ranges.shape[0] - radius_count:
                    the_min = np.min(temp_ranges[i-radius_count:])
                else:
                    the_min = np.min(temp_ranges[i-radius_count:i+radius_count+1])
                # ranges[i] = mean
                # r_index = i + self.fov_bounds[0]
                # mean = np.mean(ranges[r_index-radius_count:r_index+radius_count+1])
                limited_ranges[i] = the_min

        

        ### PRIORITISE CENTER OF SCAN ###
        limited_ranges *= self.center_priority_mask

        ### FIND DEEPEST GAP ###
        # limited_ranges = modified_ranges[self.fov_bounds[0]:self.fov_bounds[1]]
        max_gap_index = np.argmax(limited_ranges)
        goal_bearing = angle_increment * (max_gap_index - limited_ranges.shape[0] // 2)

        ### FIND TWIST ###
        init_steering = np.arctan(goal_bearing * self.wheel_base) # using ackermann steering model
        steering = self.steering_pid.update(init_steering)

        init_speed = np.sqrt(10 * self.coeffiecient_of_friction * self.wheel_base / np.abs(max(np.tan(abs(steering)),1e-16)))
        init_speed = front_clearance/self.lookahead * min(init_speed, self.speed_max)
        # init_speed = mean_range/self.lookahead * min(init_speed, self.speed_max)
        # init_speed = np.max(limited_ranges)/self.lookahead * min(init_speed, self.speed_max)
        speed = self.speed_pid.update(init_speed)

        ackermann = {"speed": speed, "steering": steering}

        ### VISUALISATION ###
        if self.visualise:
            # Visualise Modified Ranges
            self.safety_scan_msg = scan_msg
            scan_msg.ranges = modified_ranges.tolist()
            # Visualise Marked Ranges
            self.safety_markers["range"].clear()
            self.safety_markers["bearing"].clear()
            for i_range in marked_indexes:
                bearing = angle_increment * (i_range[0] - ranges.shape[0]//2)
                self.safety_markers["range"].append(i_range[1])
                self.safety_markers["bearing"].append(bearing)
            # Visualise Goal
            self.goal_marker["range"] = limited_ranges[max_gap_index]
            self.goal_marker["bearing"] = goal_bearing

        return ackermann
    
    def get_bubble_coord(self):
        m = []
        for i, r in enumerate(self.safety_markers["range"]):
            x = r * np.cos(self.safety_markers["bearing"][i])
            y = r * np.sin(self.safety_markers["bearing"][i])
            m.append([x, y])
        return m

    def get_goal_coord(self):
        x = self.goal_marker["range"] * np.cos(self.goal_marker["bearing"])
        y = self.goal_marker["range"] * np.sin(self.goal_marker["bearing"])
        return [x, y]

    def get_safety_scan(self):
        return self.safety_scan_msg


class GapFinderNode(Node):
    """
    ROS2 Node Class that handles all the subscibers and publishers for the gap finder algorithm. 
    It abstracts the gap finder algorithm from the ROS2 interface.
    The only things to tune or change are in the sections:
    - ROS2 PARAMETERS
    - SPEED AND STEERING LIMITS
    - GAP FINDER ALGORITHM
    """
    def __init__(self):
        ### ROS2 PARAMETERS ###
        self.hz = 50.0 # [Hz]
        self.timeout = 1.0 # [s]
        self.visualise = True
        scan_topic = "scan"
        drive_topic = "/nav/drive"
        # drive_topic = "drive"

        ### SPEED AND STEERING LIMITS ###
        # Speed limits
        self.max_speed = 6.0 # [m/s]
        self.min_speed = 1.0 # [m/s]
        # Acceleration limits
        self.max_acceleration = None # [m/s^2]
        # Steering limits
        self.max_steering = 0.4 # [rad]

        ### GAP FINDER ALGORITHM ###
        self.gapFinderAlgorithm = GapFinderAlgorithm(safety_bubble_diameter = 0.4, # [m] should be the width of the car
                                                     view_angle = 3.142, 
                                                     coeffiecient_of_friction = 1.0, 
                                                     disparity_threshold = 0.5,
                                                     lookahead = 3, 
                                                     speed_kp = 0.75,
                                                     steering_kp = 2.0, 
                                                     wheel_base = 0.324, 
                                                     speed_max= self.max_speed,
                                                     visualise=self.visualise)

        ### ROS2 NODE ###
        super().__init__("gap_finder")
        # Scan Subscriber
        self.scan_subscriber = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 1)
        self.scan_subscriber  # prevent unused variable warning
        self.scan_ready = False
        self.last_scan_time = self.get_time()
        # Drive Publisher
        self.drive_msg = AckermannDriveStamped()
        self.last_drive_msg = AckermannDriveStamped()
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 1)
        self.last_drive_time = self.get_time()
       # Viz Publishers
        if self.visualise:
            self.init_visualisation()
        # Timer
        self.timer = self.create_timer(1/self.hz , self.timer_callback)
    
    def init_visualisation(self):
        # Safety Viz Publisher
        self.bubble_viz_publisher = self.create_publisher(MarkerArray, "/safety_bubble", 1)
        self.bubble_viz_msg = Marker()
        self.bubble_viz_msg.header.frame_id = "/base_link"
        self.bubble_viz_msg.color.a = 1.0
        self.bubble_viz_msg.color.r = 1.0
        self.bubble_viz_msg.scale.x = self.gapFinderAlgorithm.safety_bubble_diameter
        self.bubble_viz_msg.scale.y = self.gapFinderAlgorithm.safety_bubble_diameter
        self.bubble_viz_msg.scale.z = self.gapFinderAlgorithm.safety_bubble_diameter
        self.bubble_viz_msg.type = Marker.SPHERE
        self.bubble_viz_msg.action = Marker.ADD
        # Goal Viz Publisher
        self.gap_viz_publisher = self.create_publisher(Marker, "/goal_point", 1)
        self.goal_viz_msg = Marker()
        self.goal_viz_msg.header.frame_id = "/base_link"
        self.goal_viz_msg.color.a = 1.0
        self.goal_viz_msg.color.g = 1.0
        self.goal_viz_msg.scale.x = 0.3
        self.goal_viz_msg.scale.y = 0.3
        self.goal_viz_msg.scale.z = 0.3
        self.goal_viz_msg.type = Marker.SPHERE
        self.goal_viz_msg.action = Marker.ADD
        # Laser Viz Publisher
        self.laser_publisher = self.create_publisher(LaserScan, "/safety_scan", 1)

    def get_time(self):
        """
        Returns the current time in seconds
        """
        return self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9

    def scan_callback(self, scan_msg):
        self.scan_ready = True
        self.scan_msg = scan_msg
        self.last_scan_time = self.get_time()

    def publish_drive_msg(self, drive={"speed": 0.0, "steering": 0.0}):
        self.drive_msg.drive.speed = float(drive["speed"])
        self.drive_msg.drive.steering_angle = float(drive["steering"])
        self.drive_publisher.publish(self.drive_msg)
        self.last_drive_msg = self.drive_msg
        self.last_drive_time = self.get_time()

    def publish_viz_msgs(self):
        safety_scan = self.gapFinderAlgorithm.get_safety_scan()
        bubble_coord = self.gapFinderAlgorithm.get_bubble_coord()
        goal_coord = self.gapFinderAlgorithm.get_goal_coord()
        laser_viz_msg = safety_scan
        bubble_array_viz_msg = MarkerArray()

        for i, coord in enumerate(bubble_coord):
            self.bubble_viz_msg = Marker()
            self.bubble_viz_msg.header.frame_id = "/base_link"
            self.bubble_viz_msg.color.a = 1.0
            self.bubble_viz_msg.color.r = 1.0
            self.bubble_viz_msg.scale.x = self.gapFinderAlgorithm.safety_bubble_diameter
            self.bubble_viz_msg.scale.y = self.gapFinderAlgorithm.safety_bubble_diameter
            self.bubble_viz_msg.scale.z = self.gapFinderAlgorithm.safety_bubble_diameter
            self.bubble_viz_msg.type = Marker.SPHERE
            self.bubble_viz_msg.action = Marker.ADD
            self.bubble_viz_msg.id = i
            self.bubble_viz_msg.pose.position.x = coord[0]
            self.bubble_viz_msg.pose.position.y = coord[1]
            bubble_array_viz_msg.markers.append(self.bubble_viz_msg)

        self.goal_viz_msg.pose.position.x = goal_coord[0]
        self.goal_viz_msg.pose.position.y = goal_coord[1]

        self.bubble_viz_publisher.publish(bubble_array_viz_msg)
        self.gap_viz_publisher.publish(self.goal_viz_msg)
        self.laser_publisher.publish(laser_viz_msg)


    def timer_callback(self):
        if self.scan_ready:
            ### UPDATE GAP FINDER ALGORITHM ###
            drive = self.gapFinderAlgorithm.update(self.scan_msg)

            ### APPLY SPEED AND STEERING LIMITS ###
            # steering limits
            drive["steering"] = np.sign(drive["steering"]) * min(np.abs(drive["steering"]), self.max_steering)
            # speed limits
            drive["speed"] = max(drive["speed"], self.min_speed)
            drive["speed"] = min(drive["speed"], self.max_speed)
            # acceleration limits
            if self.max_acceleration is not None:
                dt = self.get_time() - self.last_drive_time
                d_speed = drive["speed"] - self.last_drive_msg.drive.speed
                if abs(d_speed) > self.max_acceleration * dt:
                    # accelerate
                    if drive["speed"] > self.last_drive_msg.drive.speed:
                        drive["speed"] = self.last_drive_msg.drive.speed + self.max_acceleration
                    # decelerate
                    else:
                        drive["speed"] = self.last_drive_msg.drive.speed - self.max_acceleration
            # drive["speed"] = 0.0

            ### PUBLISH DRIVE MESSAGE ###
            self.publish_drive_msg(drive)
            self.last_time = self.get_time()
        
            #### PUBLISH VISUALISATION MESSAGES ###
            if self.visualise:
                self.publish_viz_msgs()

        ### TIMEOUT ###
        if ((self.get_time() - self.last_scan_time) > self.timeout):
            # self.scan_ready = False
            pass

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

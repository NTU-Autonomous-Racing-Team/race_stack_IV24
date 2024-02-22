# Ideas
# 1. Take average points to calc distance from the wall
# 2. Check if obstacle/wall is a below a certain distance in front of the car

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

import math

class WallFollow(Node):

    def __init__(self):
        super().__init__('wall_follow')

        # Subscribing to relevant topics
        self.sub_scan = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        
        self.sub_odom = self.create_subscription(
            Odometry,
            'ego_racecar/odom',
            self.odom_callback,
            10)

        self.pub_drive = self.create_publisher(
            AckermannDriveStamped,
            'drive',
            10)

        self.sub_scan
        self.sub_odom

        self.drive_msg = AckermannDriveStamped()
        
        self.Kp = 0.25
        self.Ki = 0.006
        self.Kd = 0.001
       
        self.integral = 0.0
        self.prev_error_1 = 0.0
        self.prev_secs = 0.0
        self.prev_nsecs = 0.0

        self.longitudinal_vel = 0
        self.front_dist = 0

        self.log = {"steering_angle": [], "speed": [], "actual_distance": []}

    def getRange(self, scan_data, angle):
        ranges = scan_data.ranges
        angle_rad = angle * (math.pi / 180)
        index = int( abs(angle_rad - scan_data.angle_max) / scan_data.angle_increment )

        return ranges[index]
    
    def odom_callback(self, odom_data):
        self.longitudinal_vel = odom_data.twist.twist.linear.x


    def scan_callback(self, scan_data):

        ranges = scan_data.ranges
        angle_min = scan_data.angle_min
        angle_increment = scan_data.angle_increment
        angle_max = scan_data.angle_max

        self.front_dist = self.getRange(scan_data, 0)

        secs = scan_data.header.stamp.sec
        nsecs = scan_data.header.stamp.nanosec 

        angle_b = 90
        angle_a = 40
        
        theta = (angle_b - angle_a) * (math.pi / 180)
        # 90 Degrees to the car
        distance_b = self.getRange(scan_data, angle_b) # ranges[901]
        # ~ 35 Degrees to the first scan
        distance_a = self.getRange(scan_data, angle_a) # ranges[760]
        
        alpha = -1 * math.atan2( (distance_a * math.cos(theta) - distance_b) , (distance_a * math.sin(theta)) )

        actual_distance = distance_b * math.cos(alpha)
        desired_distance = 1.2 # Metres

        error = desired_distance - actual_distance
        lookahead_distance = self.longitudinal_vel * 0.45 # Metres
        # lookahead_distance = self.longitudinal_vel * 0.15 # Metres

        error_1 = error + lookahead_distance * math.sin(alpha)
        
        if (self.prev_secs == 0.0) & (self.prev_nsecs == 0.0) & (self.prev_error_1 == 0.0):
            self.prev_secs = secs
            self.prev_nsecs = nsecs
            self.prev_error_1 = error_1

        dt = secs - self.prev_secs + (nsecs - self.prev_nsecs) * 1e-9
        
        
        if dt != 0:
            self.integral += error_1 * dt
            steering_angle = ( (self.Kp * error_1) + (self.Ki * self.integral) + (self.Kd * (error_1 - self.prev_error_1) / dt) )
            self.drive_msg.drive.steering_angle = steering_angle
            steering_angle_degrees = abs(steering_angle * (180 / math.pi))

            self.prev_error_1 = error_1
            self.prev_secs = secs
            self.prev_nsecs = nsecs
            

            # speed = 1 / exp(steering_angle_degrees)
            #self.drive_msg.drive.speed = ((steering_angle_degrees / 5.2) - 3.8)**2

            # steep drop
            # self.drive_msg.drive.speed = (1 / 2)**(steering_angle_degrees - 3.9)

            # less steep but still too much
            # self.drive_msg.drive.speed = (1 / 1.5)**(steering_angle_degrees - 6.5)
            # if self.front_dist < 2.5:
            #     self.drive_msg.drive.speed = 2.0
            # else:
            self.drive_msg.drive.speed = (1 / 1.2)**(steering_angle_degrees - 15)

            self.pub_drive.publish(self.drive_msg)
            self.get_logger().info(f"steering_angle: {steering_angle_degrees:.2f} | speed: {self.longitudinal_vel:.2f} | {actual_distance:.2f} | {lookahead_distance:.2f}")
            # self.log["steering_angle"].append(steering_angle_degrees)
            # self.log["speed"].append(self.longitudinal_vel)
            # self.log["actual_distance"].append(actual_distance)



def main(args=None):

    rclpy.init(args=args)

    wall_follow = WallFollow()

    rclpy.spin(wall_follow)

    wall_follow.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':

    main()

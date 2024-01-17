import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

from _pid import PID

# reference: https://github.com/f1tenth/f1tenth_labs_openrepo/blob/main/f1tenth_lab3/README.md

class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')
        # Laser Scans Subscriber
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.scan_subscriber  # prevent unused variable warning
        self.ranges = []
        # Odom Subscriber
        self.odom_subscriber = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.odom_subscriber
        # Drive Publisher
        self.pub_rate = 20
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.timer = self.create_timer(1/self.pub_rate, self.timer_callback)

        # Controller
        self.set_dist2wall = 1.
        self.pid = PID(0.2, 0.01, 0.)
        self.pid.set_point = self.set_dist2wall

        # WallFollower Params
        self.count = 0
        self.last_time = 0.
        self.lookahead_dist = 0.
        self.ros_time = 0.
        
        self.ttc_threshhold = 1
        self.e_brake = False
        self.linX = 0.

    def scan_init(self, scan_msg):
        self.angle_increment = scan_msg.angle_increment
        self.angle_max = scan_msg.angle_max
        self.angle_min = scan_msg.angle_min
        self.pid.integral_cutoff = self.angle_max
        self.len_range = len(scan_msg.ranges)

    def get_time(self, scan_msg):
        sec = scan_msg.header.stamp.sec
        nsec = scan_msg.header.stamp.nanosec
        ros_time = sec + nsec/1e9
        return ros_time

    def scan_callback(self, scan_msg):
        if self.count < 1:
            self.scan_init(scan_msg)
            self.count += 1
        self.ros_time = self.get_time(scan_msg)
        self.ranges = scan_msg.ranges

    def odom_callback(self, odom_msg):
        self.linX = odom_msg.twist.twist.linear.x
    
    def get_range_index(self, deg):
        angle = deg/180 * math.pi
        increments = angle/self.angle_increment
        index = round(self.len_range/2 - increments)
        return index
    
    def get_dist_at_deg(self, deg):
        index = self.get_range_index(deg)
        distance = self.ranges[index]
        return distance
    
    def find_dist2wall(self, deg_a = 89, deg_b = 90):
        theta = (deg_b - deg_a)/180 * math.pi

        dist_a = self.get_dist_at_deg(deg_a)
        dist_b = self.get_dist_at_deg(deg_b)

        alpha = math.atan((dist_a * math.cos(theta) - dist_b)/(dist_a * math.sin(theta)))
        dist2wall = dist_b * math.cos(alpha)
        pred_dist2wall = dist2wall + self.lookahead_dist * math.sin(alpha)
        return dist2wall, pred_dist2wall
    
    def clip_control_signal(self, control_signal):
        sign = 1.0
        if control_signal < 0:
            sign = -1.0
            control_signal = abs(control_signal)
        if control_signal > self.angle_max:
            control_signal = self.angle_max
        return sign * control_signal 
    
    # def clip_drive_msgs(self):


    def get_drive_msg(self, pred_dist2wall, dt):
        control_signal = self.pid.update(pred_dist2wall, dt)
        # control_signal = self.pid.update(dist2wall, dt)
        drive_msg = AckermannDriveStamped()
        if not self.e_brake:
            control_signal = self.clip_control_signal(control_signal)
            if abs(control_signal) < 1.0:
                speed = 1.6
            elif abs(control_signal) < 2.0:
                speed = 0.8
            else:
                speed = 0.4
            drive_msg.drive.speed = speed
            drive_msg.drive.steering_angle = control_signal
        else:
            print("BRAKE")
            print(drive_msg.drive.speed)
        return drive_msg

    def collision(self): #Time To Collision
        for i, radius in enumerate(self.ranges):
            angle = self.angle_max - i * self.angle_increment
            r_dot = max(-1 * self.linX * math.cos(angle), 0)
            if r_dot != 0:
                ttc = radius / r_dot
            else:
                ttc = float('inf')
            if ttc <= self.ttc_threshhold:
                self.e_brake = True
                break

    def timer_callback(self):
        if self.count >= 1:
            dt = self.ros_time - self.last_time
            dist2wall, pred_dist2wall = self.find_dist2wall(89, 90)
            self.collision()
            drive_msg = self.get_drive_msg(pred_dist2wall, dt)
            self.drive_publisher.publish(drive_msg)
            # print(f"steering_angle = {round(drive_msg.drive.steering_angle,2)}, dist2wall = {round(dist2wall, 2)}")
            # print(f"{self.e_brake}")
            self.lookahead_dist = drive_msg.drive.speed * dt
            self.last_time = self.ros_time

def main(args=None):
    rclpy.init(args=args)

    wall_follower = WallFollower()

    rclpy.spin(wall_follower)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

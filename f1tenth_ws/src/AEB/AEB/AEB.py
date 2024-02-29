import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

class AEB(Node):
    def __init__(self):
        super().__init__('AEB')

        self.time2collision = 5

        # ===[ subscribers ]=== #
        # laser Scan
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sub_scan
        self.scan_init = False
        self.ranges = []

        # odmetry
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_odom
        self.odom = []


        self.pub_rate = 20
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/aeb/drive', 10)
        self.timer = self.create_timer(1/self.pub_rate, self.timer_callback)
        self.ttc_threshhold = 1

    def scan_callback(self, scan_msg):
        if not self.scan_init:
            self.scan_angle_increment = scan_msg.angle_increment
            self.scan_angle_max = scan_msg.angle_max
            self.scan_angle_min = scan_msg.angle_min
            self.len_range = len(scan_msg.ranges)
        self.ranges = scan_msg.ranges

    def odom_callback(self, odom_msg):
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        yaw = odom_msg.pose.pose.orientation.z
        linX = odom_msg.twist.twist.linear.x
        angZ = odom_msg.twist.twist.angular.z
        self.odom = [x, y, yaw, linX, angZ]
        
    def drive_publish(self, drive_msg):
        self.drive_publisher.publish(drive_msg)

    def collision(self, ranges, linX):
        for i, radius in enumerate(ranges):
            angle = self.scan_angle_max - i * self.scan_angle_increment
            r_dot = max(-1 * linX * math.cos(angle), 0)
            if r_dot != 0:
                ttc = radius / r_dot
            else:
                ttc = float('inf')
            if ttc <= self.ttc_threshhold:
                self.e_brake = True
                break

    def timer_callback(self):
        ranges = self.ranges
        linX = self.odom[4]
        self.collision(ranges, linX)
        self.drive_publish(AckermannDriveStamped())

def main(args = None):
    rclpy.init(args=args)
    auto_e_braking = AEB()
    rclpy.spin(auto_e_braking)
    auto_e_braking.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

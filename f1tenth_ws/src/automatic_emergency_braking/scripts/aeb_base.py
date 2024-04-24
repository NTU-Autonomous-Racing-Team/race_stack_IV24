#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Float32

class AutomaticEmergencyBrakingNode(Node):
    def __init__(self, pub_rate = 10):
        super().__init__('AEB')
        # TTC
        self.ttc_subscriber = self.create_subscription(Float32, '/ttc', self.ttc_callback, 1)
        self.ttc_subscriber
        # AEB Publisher
        self.aeb_publisher = self.create_publisher(Int32, '/aeb', 1)
        self.timer = self.create_timer(1/pub_rate, self.timer_callback)
        # Memory
        self.ttc_waiting = True
        # Parameters
        self.threshold = 0.1

    def ttc_callback(self, ttc_msg):
        self.ttc_waiting = False
        self.ttc_msg = ttc_msg

    def timer_callback(self):
        if self.ttc_waiting:
            return
        # Check if TTC is less than threshold
        int_msg = Int32()
        if self.ttc_msg.data <= self.threshold:
            int_msg.data = 1
        else:
            int_msg.data = 0
        self.aeb_publisher.publish(int_msg)

def main(args = None):
    rclpy.init(args=args)
    auto_e_braking = AutomaticEmergencyBrakingNode()
    rclpy.spin(auto_e_braking)
    auto_e_braking.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()




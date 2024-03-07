#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry

import numpy as np
import pandas as pd

from os.path import expanduser
from numpy import linalg as LA
from time import gmtime, strftime

class WaypointLogger(Node):
    

    def __init__(self):
        super().__init__('waypoint_logger')
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        timer_period = 0.1
        self.ready = False
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.odom_arr = np.array([[0.0, 0.0, 0.0, 0.0]])
        self.odom_sub
        
    def timer_callback(self):    
        self.ready = True

    def odom_callback(self, odom_data):
        x = odom_data.pose.pose.position.x
        y = odom_data.pose.pose.position.y
        vel_x = odom_data.twist.twist.linear.x
        vel_y = odom_data.twist.twist.linear.y
        
        if self.ready:
            self.curr_odom_arr = np.array([[x, y, vel_x, vel_y]])
            self.odom_arr = np.vstack((self.odom_arr, self.curr_odom_arr))
            self.ready = False

def main(args=None):
    rclpy.init(args=args)

    waypoint_logger = WaypointLogger()
    try:
        rclpy.spin(waypoint_logger)
    except KeyboardInterrupt:
        odom_df = pd.DataFrame(waypoint_logger.odom_arr)
        odom_df.to_csv("~/waypoint_data.csv", header=["x", "y", "vel_x", "vel_y"], index=False)
        waypoint_logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

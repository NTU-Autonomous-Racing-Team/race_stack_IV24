#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
	
class Demux(Node):
	def __init__(self):
		super().__init__('demux')

		self.sub_scan = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
		self.sub_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
		self.sub_output_select = self.create_subscription(String, 'output_select', self.output_select_callback, 10)

		self.pub_wall_scan = self.create_publisher(LaserScan, 'wall_scan', 10)
		self.pub_gap_scan = self.create_publisher(LaserScan, 'gap_scan', 10)

		self.pub_wall_odom = self.create_publisher(Odometry, 'wall_odom', 10)
		self.pub_gap_odom = self.create_publisher(Odometry, 'gap_odom', 10)

	def scan_callback(self, scan_data):
		self.scan_data = scan_data

	def odom_callback(self, odom_data):
		self.odom_data = odom_data

	def output_select_callback(self, output_select_data):

		if output_select_data.data == "wall":
			self.pub_wall_scan.publish(self.scan_data)
			self.pub_wall_odom.publish(self.odom_data)

		elif output_select_data.data == "gap":
			self.pub_gap_scan.publish(self.scan_data)
			self.pub_gap_odom.publish(self.odom_data)

def main(args=None):
	rclpy.init(args=args)

	demux = Demux()
	rclpy.spin(demux)

	demux.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

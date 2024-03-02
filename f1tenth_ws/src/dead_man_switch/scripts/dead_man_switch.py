#!/usr/bin/env python3

import os, re
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16

class NetworkChecker():
    def __init__(self):
        self.cmd = "arp -an"
        self.status = 1
        self.regex = '^[\w\?\.]+|(?<=\s)\([\d\.]+\)|(?<=at\s)[\w\:]+|(?<=at\s)+|(?<=on\s)[\w\:]+'
        self.device = 'wlan0'

    def check_connection(self):
        results = [re.findall(self.regex, i) for i in os.popen(self.cmd)]
        results = [dict(zip(['IP', 'LAN_IP', 'MAC_ADDRESS', 'DEVICE'], i)) for i in results]
        status = 0
        for connection in results:
            if connection['DEVICE'] == self.device and connection['MAC_ADDRESS'] != '':
                status = 1
        return status
	
class DeadManSwitch(Node):
	def __init__(self):
		self.networkChecker = NetworkChecker()
		super().__init__('dead_man_switch')
		self.publisher_ = self.create_publisher(Int16, 'ssh_status', 10)
		timer_period = 0.1 # [s]
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.status = None

	def timer_callback(self):
		self.status = self.networkChecker.check_connection()
		msg = Int16()
		msg.data = self.status
		self.publisher_.publish(msg)

def main(args=None):
	rclpy.init(args=args)

	deadManSwitch = DeadManSwitch()
	rclpy.spin(deadManSwitch)

	deadManSwitch.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

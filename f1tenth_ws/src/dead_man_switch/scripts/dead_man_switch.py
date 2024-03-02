#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16

class NetworkChecker():
	def __init__(self), client_mac_address_list=None):
		self.client_mac_address_list = client_mac_address_list
		self.cmd = "arp -an"
		self.status = 1
        self.regex = '^[\w\?\.]+|(?<=\s)\([\d\.]+\)|(?<=at\s)+|(?<=on\s)[\w\:]+'

	def check_connection(self):
		results = os.popen(self.cmd)

		for mac_address in self.client_mac_address_list:
			if mac_address in
				
        results = [re.findall(self.regex, i) for i in os.popen(self.cmd)]
        results = [dict(zip(['IP', 'LAN_IP', 'MAC_ADDRESS', 'TYPE'], i)) for i in results]
        for connection in results:
            if connection['TYPE'] == 'wlan0':
                if connection['MAC_ADDRESS'] == '':
                    self.status = 0
                    
		

class DeadManSwitch(Node):
	def __init__(self):
		self.networkChecker = NetworkChecker(
		super().__init__('dead_man_swtich')
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

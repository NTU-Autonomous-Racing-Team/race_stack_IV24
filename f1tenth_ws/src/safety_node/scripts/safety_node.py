#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        # Drive Pass Through 
        self.drive_subscription = self.create_subscription(AckermannDriveStamped, 'nav/drive', self.drive_callback, 10)
        self.drive_subscription 

        self.teleop_subscription = self.create_subscription(AckermannDriveStamped, 'teleop', self.drive_callback, 10)
        self.teleop_subscription 

        self.drive_publisher = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        
        # Managed Sources
        self.ttc_subcription = self.create_subscription(Float32, 'ttc', self.ttc_callback, 10)
        self.ttc_subcription 
        self.ttc_gain = 1.0

        self.joy_subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.joy_subscription
        self.joy_gain = 1.0

        self.drive_msg = AckermannDriveStamped()
        
    def drive_callback(self, msg):
        self.drive_msg = msg
        self.drive_msg.drive.speed *= self.joy_gain * self.ttc_gain
        self.drive_publisher.publish(self.drive_msg)

    def ttc_callback(self, msg):
        if msg.data < 0.6125:
            self.get_logger().info('TTC = "%s"' % msg)
            self.ttc_gain = 0.0

    def joy_callback(self, msg):
        buttons_list = msg.buttons
        # dead_man_switch = buttons_list[4] # DualShock4 L1
        dead_man_switch = buttons_list[5] # DualShock4 R1
        if dead_man_switch:
            self.joy_gain = 1.0
        else:
            self.joy_gain = 0.0
            self.ttc_gain = 1.0

def main(args = None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

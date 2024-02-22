import math
import numpy as np
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class TestSpeed2ErpmGain(Node):
    def __init__(self):
        super().__init__('odom_calibration')
        self.publish_period = 0.05  # [s]
        self.max_time = 5 # [s]
        self.max_linX = 1 # [m/s]
        self.acceleration = 0.5 # [m/s^2]
        self.displacement = 0 # [m]
        self.last_linX = 0 # [m/s]
        self.last_time = 0 # [s]

        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.timer = self.create_timer(self.publish_period, self.timer_callback)
        self.start_time = self.get_clock().now().nanoseconds

    def timer_callback(self):
      if (self.get_clock().now().nanoseconds - self.start_time)/1e9 <= self.max_time:
        twist_msg = self.get_twist_msg()
        self.publisher_.publish(twist_msg)
      else:
        self.get_logger().info("Finished")
        self.get_logger().info(f"Expected: {self.displacement}")
        print(f"Expected: {self.displacement}")
        self.timer.cancel()

    def get_twist_msg(self):
      current_time = (self.get_clock().now().nanoseconds - self.start_time)/1e9
      if current_time < self.max_time/3 and self.last_linX < self.max_linX:
        linX = self.last_linX + self.acceleration * (current_time - self.last_time)
      elif current_time < 2 * self.max_time/3:
        linX = self.last_linX
      elif current_time < self.max_time:
        linX = self.last_linX - self.acceleration * (current_time - self.last_time)
      else:
        linX = 0
      twist = AckermannDriveStamped()
      twist.drive.speed = linX
      self.displacement += linX * (current_time - self.last_time)
      self.last_time = current_time
      self.last_linX = linX
      return twist

def main(args=None):
    rclpy.init(args=args)
    test_speed_to_erpm_gain = TestSpeed2ErpmGain()
    rclpy.spin(test_speed_to_erpm_gain)
    test_speed_to_erpm_gain.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
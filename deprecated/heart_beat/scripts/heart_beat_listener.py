import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8
from ackermann_msgs.msg import AckermannDriveStamped

class HeartBeatListener(Node):

    def __init__(self):
        super().__init__('heart_beat_listener')
        self.subscription = self.create_subscription(
            Int8,
            'heart_beat',
            self.heart_beat_callback,
            1)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.last_heart_beat = self.get_time()

    def get_time(self):
        return self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9

    def heart_beat_callback(self, msg):
        self.last_heart_beat = self.get_time()

    def timer_callback(self):
        if self.get_time() - self.last_heart_beat > 1:
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = 0.0
            drive_msg.drive.steering_angle = 0.0
            self.publisher_.publish(drive_msg)
        

def main(args=None):
    rclpy.init(args=args)

    heart_beat_listenter = HeartBeatListener()

    rclpy.spin(heart_beat_listenter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    heart_beat_listenter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
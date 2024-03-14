import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8


class HeartBeatPublisher(Node):

    def __init__(self):
        super().__init__('heart_beat_publisher')
        self.publisher_ = self.create_publisher(Int8, 'heart_beat', 1)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Int8()
        msg.data = 1
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    heart_beat_publisher = HeartBeatPublisher()

    rclpy.spin(heart_beat_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    heart_beat_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
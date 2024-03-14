import rclpy
import rclpy.node as Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        self.drive_subscription = self.create_subscription(
            AckermannDriveStamped,
            'drive',
            self.drive_callback,
            10)
        self.drive_subscription 

        self.ttc_subcription = self.create_subscription(
            Float32,
            'ttc',
            self.ttc_callback,
            10)
        self.ttc_subcription 

        self.teleop_subscription = self.create_subscription(
            AckermannDriveStamped, 
            'teleop', 
            self.teleop_callback,
            10)
        self.teleop_subscription

        self.drive_publisher = self.create_publisher(AckermannDriveStamped, 'drive', 10)

        self.drive_msg = AckermannDriveStamped()
        self.speed_gain = 1.0
        
    def drive_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg)
        self.drive_msg = msg* self.speed_gain
        self.drive_publisher.publish(self.drive_msg)

    def ttc_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg)
        if msg.data < 0.5:
            self.speed_gain = 0.0

    def teleop_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg)
        self.speed_gain = 1.0
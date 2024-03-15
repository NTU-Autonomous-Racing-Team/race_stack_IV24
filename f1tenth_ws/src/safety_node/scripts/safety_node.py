import rclpy
import rclpy.node as Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        # Drive Pass Through 
        self.drive_subscription = self.create_subscription(
            AckermannDriveStamped,
            'drive',
            self.drive_callback,
            10)
        self.drive_subscription 

        self.drive_publisher = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        
        # Managed Sources
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
        self.last_teleop = self.get_time()

        self.drive_msg = AckermannDriveStamped()
        self.speed_gain = 1.0
    
    def get_time(self):
        return self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        
    def drive_callback(self, msg):
        self.drive_msg = msg* self.speed_gain
        self.drive_publisher.publish(self.drive_msg)

    def ttc_callback(self, msg):
        if msg.data < 0.5:
            self.get_logger().info('TTC = "%s"' % msg)
            self.speed_gain = 0.0
        else:
            self.speed_gain = 1.0

    def teleop_callback(self, msg):
        if (self.get_time() - self.last_teleop) < 1:
            self.speed_gain = 1.0
        else:
            self.get_logger().info('Teleop not publishing for past 1 second, stopping car.')
            self.speed_gain = 0.0
        self.last_teleop = self.get_time()
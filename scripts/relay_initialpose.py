import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped

class Relay(Node):
    def __init__(self):
        super().__init__('initialpose_relay')
        subq = QoSProfile(depth=10); subq.reliability = ReliabilityPolicy.BEST_EFFORT
        pubq = QoSProfile(depth=10); pubq.reliability = ReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/rtabmap/initialpose', pubq)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.cb, subq)
        self.get_logger().info('initialpose relay up: /initialpose -> /rtabmap/initialpose')
    def cb(self, msg):
        self.pub.publish(msg)
        self.get_logger().info('relayed initial pose')

rclpy.init(); rclpy.spin(Relay())

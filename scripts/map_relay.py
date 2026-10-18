#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid

class MapRelay(Node):
    def __init__(self):
        super().__init__("map_relay")
        sub_qos = QoSProfile(depth=1)
        sub_qos.reliability = ReliabilityPolicy.RELIABLE
        sub_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        sub_qos.history = HistoryPolicy.KEEP_LAST
        pub_qos = QoSProfile(depth=1)
        pub_qos.reliability = ReliabilityPolicy.RELIABLE
        pub_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        pub_qos.history = HistoryPolicy.KEEP_LAST
        self.pub = self.create_publisher(OccupancyGrid, "/map", pub_qos)
        self.create_subscription(OccupancyGrid, "/rtabmap/map", self.cb, sub_qos)
        self.latest = None
        self.create_timer(1.0, self.tick)
        self.get_logger().info("map_relay up: /rtabmap/map (transient_local) -> /map (republish 1Hz, latched)")
    def cb(self, msg):
        self.latest = msg
    def tick(self):
        if self.latest is not None:
            self.pub.publish(self.latest)

def main():
    rclpy.init()
    rclpy.spin(MapRelay())

if __name__ == "__main__":
    main()

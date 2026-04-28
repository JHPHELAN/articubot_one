#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion
import math

def get_quaternion_from_yaw(yaw):
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0)
    )

class AnchorDisplay(Node):
    def __init__(self):
        super().__init__('anchor_display')
        self.marker_pub = self.create_publisher(MarkerArray, '/anchor_markers', 10)
        self.timer = self.create_timer(1.0, self.publish_anchors)
        
        # Define anchors here (x, y, yaw in radians)
        self.anchors = [
            {"id": 0, "name": "dock", "x": 0.0, "y": 0.0, "yaw": 0.0},
            {"id": 1, "name": "study_door", "x": 1.32, "y": 1.32, "yaw": math.radians(45.0)},
            {"id": 2, "name": "foyer/front door", "x": -1.76, "y": 2.63, "yaw": 0.0},
            {"id": 3, "name": "guest suite", "x": 1.11, "y": 3.07, "yaw": math.pi/2.0},
            {"id": 4, "name": "hallway", "x": 2.64, "y": 0.876, "yaw": -math.pi/2.0},
            {"id": 5, "name": "dining", "x": 6.61, "y": 0.432, "yaw": 0.0},
            {"id": 6, "name": "kitchen", "x": 11.0, "y": 0.877, "yaw": 0.0},
            {"id": 7, "name": "island", "x": 13.2, "y": -2.21, "yaw": 0.0},
            {"id": 8, "name": "living", "x": 15.8, "y": 0.0, "yaw": 0.0},
            {"id": 9, "name": "master", "x": 20.7, "y": 0.0, "yaw": -math.pi/2.0},
            {"id": 10, "name": "bathroom", "x": 17.2, "y": -5.72, "yaw": math.pi},
            {"id": 11, "name": "closet", "x": 11.9, "y": -5.5, "yaw": math.pi}
        ]

    def publish_anchors(self):
        marker_array = MarkerArray()
        
        for anchor in self.anchors:
            # 1. Text Label Marker
            txt_marker = Marker()
            txt_marker.header.frame_id = "map"
            txt_marker.header.stamp = self.get_clock().now().to_msg()
            txt_marker.ns = "anchor_labels"
            txt_marker.id = anchor["id"]
            txt_marker.type = Marker.TEXT_VIEW_FACING
            txt_marker.action = Marker.ADD
            txt_marker.pose.position.x = anchor["x"]
            txt_marker.pose.position.y = anchor["y"]
            txt_marker.pose.position.z = 0.3 # float slightly above
            txt_marker.pose.orientation.x = 0.0
            txt_marker.pose.orientation.y = 0.0
            txt_marker.pose.orientation.z = 0.0
            txt_marker.pose.orientation.w = 1.0
            txt_marker.scale.z = 0.2 # Text height (increased)
            txt_marker.color.a = 1.0 # Alpha
            txt_marker.color.r = 0.0
            txt_marker.color.g = 0.5 # Bright Blue
            txt_marker.color.b = 1.0
            txt_marker.text = str(anchor['id'])
            marker_array.markers.append(txt_marker)

            # 2. Arrow Marker
            arrow_marker = Marker()
            arrow_marker.header.frame_id = "map"
            arrow_marker.header.stamp = self.get_clock().now().to_msg()
            arrow_marker.ns = "anchor_arrows"
            arrow_marker.id = anchor["id"] + 1000 # Offset ID to avoid collision
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.pose.position.x = anchor["x"]
            arrow_marker.pose.position.y = anchor["y"]
            arrow_marker.pose.position.z = 0.0
            arrow_marker.pose.orientation = get_quaternion_from_yaw(anchor["yaw"])
            arrow_marker.scale.x = 0.4 # Arrow length
            arrow_marker.scale.y = 0.05 # Arrow width
            arrow_marker.scale.z = 0.05 # Arrow height
            arrow_marker.color.a = 0.8
            arrow_marker.color.r = 0.0
            arrow_marker.color.g = 1.0 # Green arrows
            arrow_marker.color.b = 0.0
            marker_array.markers.append(arrow_marker)

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = AnchorDisplay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

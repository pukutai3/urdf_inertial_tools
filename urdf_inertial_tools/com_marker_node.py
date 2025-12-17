#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker


class CenterOfMassMarker(Node):

    def __init__(self):
        super().__init__('center_of_mass_marker')

        # Declare parameters
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)

        # Publisher
        self.publisher = self.create_publisher(Marker, 'center_of_mass', 10)

        # Timer (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_marker)

    def publish_marker(self):
        marker = Marker()

        marker.header.frame_id = self.get_parameter('frame_id').value
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'center_of_mass'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Position (center of mass)
        marker.pose.position.x = self.get_parameter('x').value
        marker.pose.position.y = self.get_parameter('y').value
        marker.pose.position.z = self.get_parameter('z').value
        marker.pose.orientation.w = 1.0

        # Sphere size (diameter = 0.02 m)
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02

        # Color (red)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.publisher.publish(marker)


def main():
    rclpy.init()
    node = CenterOfMassMarker()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


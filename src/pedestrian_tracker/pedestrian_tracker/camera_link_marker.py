#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker


class CameraLinkMarker(Node):

    def __init__(self):
        super().__init__('camera_link_marker')

        self.publisher = self.create_publisher(
            Marker,
            '/camera_link_marker',
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.publish_marker
        )

        self.get_logger().info(
            'Publishing camera marker on /camera_link_marker'
        )

    def publish_marker(self):

        marker = Marker()

        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'camera_link'

        marker.ns = 'camera'
        marker.id = 0

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.20
        marker.scale.y = 0.20
        marker.scale.z = 0.20

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.publisher.publish(marker)


def main(args=None):

    rclpy.init(args=args)

    node = CameraLinkMarker()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

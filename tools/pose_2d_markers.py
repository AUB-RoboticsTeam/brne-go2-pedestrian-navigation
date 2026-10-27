#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
)

from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from yolo_msgs.msg import DetectionArray


# COCO pose skeleton.
# yolo_ros is giving us 1-based keypoint IDs (1..17).
SKELETON = [
    (1, 2), (1, 3),
    (2, 4), (3, 5),

    (6, 7),

    (6, 8), (8, 10),
    (7, 9), (9, 11),

    (6, 12), (7, 13),
    (12, 13),

    (12, 14), (14, 16),
    (13, 15), (15, 17),
]


class Pose2DMarkerNode(Node):

    def __init__(self):
        super().__init__('pose_2d_marker_node')

        # Put the 2D skeleton on a virtual plane 2 m
        # in front of the camera.
        self.z_plane = 2.0

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.pose_count = 0
        self.info_received = False

        # -----------------------------------------------------
        # Publisher
        # -----------------------------------------------------
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/yolo/pose_2d_markers',
            10
        )

        # -----------------------------------------------------
        # Camera intrinsics
        # -----------------------------------------------------
        self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_cb,
            qos_profile_sensor_data
        )

        # -----------------------------------------------------
        # Tracked YOLO poses
        # -----------------------------------------------------
        yolo_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.create_subscription(
            DetectionArray,
            '/yolo/tracking',
            self.detections_cb,
            yolo_qos
        )

        # Print status every 2 seconds
        self.create_timer(2.0, self.status_cb)

        self.get_logger().info('2D pedestrian pose marker node started')
        self.get_logger().info(
            'Publishing: /yolo/pose_2d_markers'
        )

    def camera_info_cb(self, msg):

        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

        if not self.info_received:
            self.info_received = True

            self.get_logger().info(
                f'Camera intrinsics received: '
                f'fx={self.fx:.1f}, fy={self.fy:.1f}, '
                f'cx={self.cx:.1f}, cy={self.cy:.1f}'
            )

    def pixel_to_3d_plane(self, u, v):

        z = self.z_plane

        p = Point()

        # camera_color_optical_frame:
        # X = right
        # Y = down
        # Z = forward
        p.x = (float(u) - self.cx) * z / self.fx
        p.y = (float(v) - self.cy) * z / self.fy
        p.z = z

        return p

    def detections_cb(self, msg):

        if self.fx is None:
            return

        marker_array = MarkerArray()

        # Clear previous skeletons
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        person_number = 0

        for detection in msg.detections:

            if detection.class_name != 'person':
                continue

            if len(detection.keypoints.data) == 0:
                continue

            person_number += 1

            # Convert keypoints into points on virtual plane
            points = {}

            for kp in detection.keypoints.data:
                points[kp.id] = self.pixel_to_3d_plane(
                    kp.point.x,
                    kp.point.y
                )

            # =================================================
            # JOINT MARKERS
            # =================================================

            joints = Marker()

            joints.header.stamp = msg.header.stamp
            joints.header.frame_id = 'camera_color_optical_frame'

            joints.ns = f'person_{person_number}_joints'
            joints.id = 0

            joints.type = Marker.SPHERE_LIST
            joints.action = Marker.ADD

            joints.scale.x = 0.05
            joints.scale.y = 0.05
            joints.scale.z = 0.05

            joints.color.r = 1.0
            joints.color.g = 0.2
            joints.color.b = 0.2
            joints.color.a = 1.0

            joints.points = list(points.values())

            marker_array.markers.append(joints)

            # =================================================
            # SKELETON/BONES
            # =================================================

            bones = Marker()

            bones.header.stamp = msg.header.stamp
            bones.header.frame_id = 'camera_color_optical_frame'

            bones.ns = f'person_{person_number}_bones'
            bones.id = 0

            bones.type = Marker.LINE_LIST
            bones.action = Marker.ADD

            bones.scale.x = 0.035

            bones.color.r = 0.2
            bones.color.g = 1.0
            bones.color.b = 0.2
            bones.color.a = 1.0

            for a, b in SKELETON:
                if a in points and b in points:
                    bones.points.append(points[a])
                    bones.points.append(points[b])

            marker_array.markers.append(bones)

        self.pose_count = person_number

        # Publish even if there are zero people, so old markers
        # are cleared from RViz.
        self.marker_pub.publish(marker_array)

    def status_cb(self):

        self.get_logger().info(
            f'camera_info={self.info_received}, '
            f'persons={self.pose_count}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = Pose2DMarkerNode()

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

#!/usr/bin/env python3

import math
import zlib
from collections import deque

import numpy as np

import rclpy
from rclpy.duration import Duration as RclpyDuration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from yolo_msgs.msg import DetectionArray

from crowd_nav_interfaces.msg import (
    Pedestrian,
    PedestrianArray,
)

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from geometry_msgs.msg import (
    Point,
    Pose,
    PoseArray,
    TransformStamped,
)
from nav_msgs.msg import Odometry

from builtin_interfaces.msg import Duration

from tf2_ros import (
    Buffer,
    StaticTransformBroadcaster,
    TransformException,
    TransformListener,
)


def quaternion_multiply(left, right):
    """Return Hamilton product ``left * right`` as [x, y, z, w]."""
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return np.array([
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ], dtype=float)


def quaternion_yaw(quaternion):
    """Extract planar yaw from an [x, y, z, w] quaternion."""
    x, y, z, w = quaternion
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def extrapolate_planar_camera_pose(
    camera_position,
    camera_quaternion,
    base_position,
    base_quaternion,
    body_linear_velocity,
    yaw_rate,
    dt,
):
    """Advance a camera pose with planar rigid-body translation and yaw.

    The rotational compensation is ``delta_yaw = yaw_rate * dt``.  The
    camera lever arm is rotated about the robot base, so an offset camera is
    handled correctly during an in-place turn.
    """
    delta_yaw = float(yaw_rate) * float(dt)
    cosine = math.cos(delta_yaw)
    sine = math.sin(delta_yaw)

    camera_position = np.asarray(camera_position, dtype=float)
    base_position = np.asarray(base_position, dtype=float)
    lever_arm = camera_position - base_position
    rotated_lever_arm = np.array([
        cosine * lever_arm[0] - sine * lever_arm[1],
        sine * lever_arm[0] + cosine * lever_arm[1],
        lever_arm[2],
    ])

    base_yaw = quaternion_yaw(base_quaternion)
    vx, vy, vz = np.asarray(body_linear_velocity, dtype=float)
    world_velocity = np.array([
        math.cos(base_yaw) * vx - math.sin(base_yaw) * vy,
        math.sin(base_yaw) * vx + math.cos(base_yaw) * vy,
        vz,
    ])
    advanced_position = (
        base_position + world_velocity * float(dt) + rotated_lever_arm
    )

    yaw_delta_quaternion = np.array([
        0.0,
        0.0,
        math.sin(delta_yaw / 2.0),
        math.cos(delta_yaw / 2.0),
    ])
    advanced_quaternion = quaternion_multiply(
        yaw_delta_quaternion,
        np.asarray(camera_quaternion, dtype=float),
    )
    norm = np.linalg.norm(advanced_quaternion)
    if norm > 0.0:
        advanced_quaternion /= norm
    return advanced_position, advanced_quaternion


class PedestrianStateNode(Node):

    def __init__(self):

        super().__init__('pedestrian_state_node')

        # =============================================================
        # PARAMETERS
        # =============================================================

        self.declare_parameter(
            'input_topic',
            '/yolo/detections_3d'
        )

        # Kalman filter
        self.declare_parameter(
            'measurement_std',
            0.08
        )

        self.declare_parameter(
            'acceleration_std',
            1.0
        )

        self.declare_parameter(
            'mahalanobis_gate',
            9.21
        )

        self.declare_parameter(
            'max_consecutive_rejections',
            5
        )

        self.declare_parameter(
            'track_timeout',
            0.80
        )

        self.declare_parameter(
            'minimum_track_hits',
            2
        )

        # =============================================================
        # VELOCITY / ARROW
        # =============================================================

        # Hysteresis:
        #
        # OFF -> ON above 0.18 m/s
        # ON  -> OFF below 0.10 m/s

        self.declare_parameter(
            'arrow_on_speed',
            0.18
        )

        self.declare_parameter(
            'arrow_off_speed',
            0.10
        )

        self.declare_parameter(
            'maximum_speed',
            2.5
        )

        self.declare_parameter(
            'stationary_speed_threshold',
            0.15
        )

        self.declare_parameter(
            'velocity_arrow_scale',
            1.0
        )

        self.declare_parameter(
            'maximum_arrow_length',
            2.0
        )

        # =============================================================
        # POSITION
        # =============================================================

        self.declare_parameter(
            'z_smoothing',
            0.0
        )

        self.declare_parameter(
            'tf_lookup_timeout',
            0.08
        )

        self.declare_parameter(
            'max_tf_future_tolerance',
            0.075
        )

        self.declare_parameter(
            'source_stamp_clock_tolerance',
            5.0
        )

        # Marker remains briefly if YOLO misses a frame.
        self.declare_parameter(
            'marker_lifetime',
            1.0
        )

        # =============================================================
        # BRNE OUTPUT
        # =============================================================

        self.declare_parameter(
            'brne_pedestrian_topic',
            '/pedestrians'
        )

        self.declare_parameter(
            'planning_frame',
            'brne_odom'
        )

        self.declare_parameter(
            'odom_topic',
            '/brne_odom'
        )

        # =============================================================
        # STATIC CAMERA TF
        # =============================================================

        self.declare_parameter(
            'publish_camera_static_tf',
            True
        )

        self.declare_parameter(
            'camera_parent_frame',
            'base_link'
        )

        self.declare_parameter(
            'camera_frame',
            'camera_link'
        )

        self.declare_parameter(
            'camera_x',
            0.34
        )

        self.declare_parameter(
            'camera_y',
            0.0
        )

        self.declare_parameter(
            'camera_z',
            0.08
        )

        self.declare_parameter(
            'camera_upward_pitch_deg',
            30.0
        )

        # =============================================================
        # READ PARAMETERS
        # =============================================================

        self.input_topic = self.get_parameter(
            'input_topic'
        ).value

        self.measurement_std = float(
            self.get_parameter(
                'measurement_std'
            ).value
        )

        self.acceleration_std = float(
            self.get_parameter(
                'acceleration_std'
            ).value
        )

        self.mahalanobis_gate = float(
            self.get_parameter(
                'mahalanobis_gate'
            ).value
        )

        self.max_consecutive_rejections = int(
            self.get_parameter(
                'max_consecutive_rejections'
            ).value
        )

        self.track_timeout = float(
            self.get_parameter(
                'track_timeout'
            ).value
        )

        self.minimum_track_hits = int(
            self.get_parameter(
                'minimum_track_hits'
            ).value
        )

        self.arrow_on_speed = float(
            self.get_parameter(
                'arrow_on_speed'
            ).value
        )

        self.arrow_off_speed = float(
            self.get_parameter(
                'arrow_off_speed'
            ).value
        )

        self.maximum_speed = float(
            self.get_parameter(
                'maximum_speed'
            ).value
        )

        self.stationary_speed_threshold = float(
            self.get_parameter(
                'stationary_speed_threshold'
            ).value
        )

        self.arrow_scale = float(
            self.get_parameter(
                'velocity_arrow_scale'
            ).value
        )

        self.maximum_arrow_length = float(
            self.get_parameter(
                'maximum_arrow_length'
            ).value
        )

        self.z_smoothing = float(
            self.get_parameter(
                'z_smoothing'
            ).value
        )

        self.tf_lookup_timeout = float(
            self.get_parameter(
                'tf_lookup_timeout'
            ).value
        )

        self.max_tf_future_tolerance = float(
            self.get_parameter(
                'max_tf_future_tolerance'
            ).value
        )

        self.source_stamp_clock_tolerance = float(
            self.get_parameter(
                'source_stamp_clock_tolerance'
            ).value
        )

        marker_lifetime_seconds = float(
            self.get_parameter(
                'marker_lifetime'
            ).value
        )

        marker_sec = int(
            marker_lifetime_seconds
        )

        marker_nanosec = int(
            (
                marker_lifetime_seconds
                - marker_sec
            ) * 1e9
        )

        self.marker_lifetime = Duration(
            sec=marker_sec,
            nanosec=marker_nanosec
        )

        self.brne_pedestrian_topic = self.get_parameter(
            'brne_pedestrian_topic'
        ).value

        self.planning_frame = self.get_parameter(
            'planning_frame'
        ).value

        self.odom_topic = self.get_parameter(
            'odom_topic'
        ).value

        self.camera_frame = self.get_parameter(
            'camera_frame'
        ).value

        # =============================================================
        # KALMAN FILTER
        #
        # state:
        #
        # [x, y, vx, vy]
        #
        # The state is expressed in the fixed planning frame. Filtering
        # camera-relative coordinates would incorrectly turn robot motion
        # into apparent pedestrian motion.
        # =============================================================

        self.H = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
            ],
            dtype=float
        )

        self.R = (
            np.eye(2)
            * self.measurement_std ** 2
        )

        self.I4 = np.eye(4)

        # =============================================================
        # TRACKS
        # =============================================================

        self.tracks = {}

        # Camera device timestamps and the local ROS clock may have different
        # epochs. Estimate their offset from detection arrival times while
        # retaining the source timestamp spacing used by the Kalman filter.
        self.source_time_offsets_ns = deque(maxlen=100)

        # A short timestamped odometry history supports rotation-aware
        # extrapolation when a camera observation is slightly newer than the
        # most recent TF sample.
        self.odom_history = deque(maxlen=100)

        # BRNE's C++ implementation uses pedestrian IDs as vector
        # indices, so expose compact integer IDs instead of YOLO's
        # arbitrary string track IDs.
        self.brne_ids = {}
        self.free_brne_ids = []
        self.next_brne_id = 0

        # =============================================================
        # STATIC TF
        # =============================================================

        self.static_tf_broadcaster = (
            StaticTransformBroadcaster(self)
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        if bool(
            self.get_parameter(
                'publish_camera_static_tf'
            ).value
        ):

            self.publish_camera_static_transform()

        # =============================================================
        # ROS INTERFACES
        # =============================================================

        self.subscription = self.create_subscription(
            DetectionArray,
            self.input_topic,
            self.detection_callback,
            qos_profile_sensor_data
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/pedestrians/markers',
            10
        )

        self.pose_pub = self.create_publisher(
            PoseArray,
            '/pedestrians/poses',
            10
        )

        self.brne_pedestrian_pub = self.create_publisher(
            PedestrianArray,
            self.brne_pedestrian_topic,
            10
        )

        self.get_logger().info(
            f'YOLO input: {self.input_topic}'
        )

        self.get_logger().info(
            f'Raw detection frame: {self.camera_frame}; '
            f'filtered output frame: {self.planning_frame}'
        )

        self.get_logger().info(
            f'BRNE pedestrians: {self.brne_pedestrian_topic} '
            f'in {self.planning_frame}'
        )

        self.get_logger().info(
            'Kalman state: [x, y, vx, vy]'
        )

        self.get_logger().info(
            f'Arrow hysteresis: '
            f'ON>{self.arrow_on_speed:.2f} m/s, '
            f'OFF<{self.arrow_off_speed:.2f} m/s'
        )

        self.get_logger().info(
            f'Minimum track hits: {self.minimum_track_hits}; '
            f'stationary threshold: '
            f'{self.stationary_speed_threshold:.2f} m/s'
        )

    # =================================================================
    # STATIC TF
    # =================================================================

    def publish_camera_static_transform(self):

        parent = self.get_parameter(
            'camera_parent_frame'
        ).value

        child = self.get_parameter(
            'camera_frame'
        ).value

        x = float(
            self.get_parameter('camera_x').value
        )

        y = float(
            self.get_parameter('camera_y').value
        )

        z = float(
            self.get_parameter('camera_z').value
        )

        upward_pitch = math.radians(float(
            self.get_parameter(
                'camera_upward_pitch_deg'
            ).value
        ))

        transform = TransformStamped()

        transform.header.stamp = (
            self.get_clock().now().to_msg()
        )

        transform.header.frame_id = parent
        transform.child_frame_id = child

        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z

        # camera_link uses +X forward and +Z up. In ROS RPY convention,
        # a physically upward camera tilt is a negative rotation about Y.
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = math.sin(-upward_pitch / 2.0)
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = math.cos(upward_pitch / 2.0)

        self.static_tf_broadcaster.sendTransform(
            transform
        )

        self.get_logger().info(
            f'Static TF: {parent} -> {child} | '
            f'xyz=({x:.3f}, {y:.3f}, {z:.3f}), '
            f'upward_pitch={math.degrees(upward_pitch):.1f} deg'
        )

    # =================================================================
    # MARKER ID
    # =================================================================

    def marker_id_from_track_id(
        self,
        pedestrian_id
    ):

        return (
            zlib.crc32(
                pedestrian_id.encode('utf-8')
            )
            & 0x7FFFFFFF
        )

    def brne_id_from_track_id(
        self,
        pedestrian_id
    ):

        if pedestrian_id not in self.brne_ids:

            if self.free_brne_ids:

                brne_id = self.free_brne_ids.pop(0)

            else:

                brne_id = self.next_brne_id
                self.next_brne_id += 1

            self.brne_ids[pedestrian_id] = brne_id

        return self.brne_ids[pedestrian_id]

    @staticmethod
    def rotate_vector(
        vector,
        quaternion
    ):

        # Quaternion-vector rotation without an additional dependency.
        q = np.array(
            [
                quaternion.x,
                quaternion.y,
                quaternion.z,
            ],
            dtype=float
        )

        v = np.array(vector, dtype=float)
        t = 2.0 * np.cross(q, v)

        return (
            v
            + quaternion.w * t
            + np.cross(q, t)
        )

    # =================================================================
    # NEW TRACK
    # =================================================================

    def create_track(
        self,
        x,
        y,
        z,
        timestamp
    ):

        state = np.array(
            [
                x,
                y,
                0.0,
                0.0
            ],
            dtype=float
        )

        covariance = np.diag(
            [
                self.measurement_std ** 2,
                self.measurement_std ** 2,
                0.7 ** 2,
                0.7 ** 2
            ]
        )

        return {
            'x': state,
            'P': covariance,

            'last_time': timestamp,
            'last_seen': timestamp,

            'z': z,

            'heading': 0.0,

            'reject_count': 0,

            # Suppress isolated one-frame tracker/depth glitches.
            'hits': 1,

            # Hysteresis state
            'arrow_visible': False,
        }

    # =================================================================
    # MOTION MODEL
    # =================================================================

    def build_motion_model(
        self,
        dt
    ):

        F = np.array(
            [
                [1.0, 0.0, dt, 0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float
        )

        G = np.array(
            [
                [0.5 * dt * dt, 0.0],
                [0.0, 0.5 * dt * dt],
                [dt, 0.0],
                [0.0, dt],
            ],
            dtype=float
        )

        Q = (
            G
            @ G.T
            * self.acceleration_std ** 2
        )

        return F, Q

    # =================================================================
    # KALMAN FILTER
    # =================================================================

    def update_track(
        self,
        pedestrian_id,
        measurement_x,
        measurement_y,
        measurement_z,
        timestamp
    ):

        if pedestrian_id not in self.tracks:

            self.tracks[pedestrian_id] = (
                self.create_track(
                    measurement_x,
                    measurement_y,
                    measurement_z,
                    timestamp
                )
            )

            return self.tracks[
                pedestrian_id
            ]

        track = self.tracks[
            pedestrian_id
        ]

        dt = (
            timestamp
            - track['last_time']
        )

        if (
            dt <= 0.0
            or dt > self.track_timeout
        ):

            self.tracks[pedestrian_id] = (
                self.create_track(
                    measurement_x,
                    measurement_y,
                    measurement_z,
                    timestamp
                )
            )

            return self.tracks[
                pedestrian_id
            ]

        # =============================================================
        # PREDICT
        # =============================================================

        F, Q = self.build_motion_model(
            dt
        )

        x_pred = (
            F
            @ track['x']
        )

        P_pred = (
            F
            @ track['P']
            @ F.T
            + Q
        )

        # =============================================================
        # MEASUREMENT
        # =============================================================

        measurement = np.array(
            [
                measurement_x,
                measurement_y
            ],
            dtype=float
        )

        innovation = (
            measurement
            - self.H @ x_pred
        )

        S = (
            self.H
            @ P_pred
            @ self.H.T
            + self.R
        )

        try:

            inv_S = np.linalg.inv(
                S
            )

        except np.linalg.LinAlgError:

            inv_S = np.linalg.pinv(
                S
            )

        # =============================================================
        # OUTLIER REJECTION
        # =============================================================

        mahalanobis_squared = float(
            innovation.T
            @ inv_S
            @ innovation
        )

        if (
            mahalanobis_squared
            > self.mahalanobis_gate
        ):

            track['reject_count'] += 1

            track['x'] = x_pred
            track['P'] = P_pred

            track['last_time'] = timestamp
            track['last_seen'] = timestamp

            if (
                track['reject_count']
                >= self.max_consecutive_rejections
            ):

                self.tracks[pedestrian_id] = (
                    self.create_track(
                        measurement_x,
                        measurement_y,
                        measurement_z,
                        timestamp
                    )
                )

                return self.tracks[
                    pedestrian_id
                ]

            return track

        # =============================================================
        # CORRECT
        # =============================================================

        K = (
            P_pred
            @ self.H.T
            @ inv_S
        )

        x_new = (
            x_pred
            + K @ innovation
        )

        KH = (
            K
            @ self.H
        )

        P_new = (
            (self.I4 - KH)
            @ P_pred
            @ (self.I4 - KH).T

            + K
            @ self.R
            @ K.T
        )

        # =============================================================
        # SPEED LIMIT
        # =============================================================

        speed = math.hypot(
            x_new[2],
            x_new[3]
        )

        if speed > self.maximum_speed:

            scale = (
                self.maximum_speed
                / speed
            )

            x_new[2] *= scale
            x_new[3] *= scale

        speed = math.hypot(
            x_new[2],
            x_new[3]
        )

        # =============================================================
        # ARROW HYSTERESIS
        # =============================================================

        if track['arrow_visible']:

            if speed <= self.arrow_off_speed:

                track['arrow_visible'] = False

        else:

            if speed >= self.arrow_on_speed:

                track['arrow_visible'] = True

        # =============================================================
        # HEADING
        #
        # Only update direction when speed is reliable.
        # =============================================================

        if speed >= self.arrow_on_speed:

            track['heading'] = math.atan2(
                x_new[3],
                x_new[2]
            )

        # =============================================================
        # Z FILTER
        # =============================================================

        filtered_z = (
            self.z_smoothing
            * track['z']

            + (
                1.0
                - self.z_smoothing
            )
            * measurement_z
        )

        # =============================================================
        # SAVE
        # =============================================================

        track['x'] = x_new
        track['P'] = P_new

        track['last_time'] = timestamp
        track['last_seen'] = timestamp

        track['z'] = filtered_z

        track['reject_count'] = 0
        track['hits'] += 1

        return track

    # =================================================================
    # CALLBACK
    # =================================================================

    def odom_callback(self, msg):
        """Retain robot twist and base pose for short TF extrapolations."""
        self.odom_history.append(msg)

    def extrapolate_latest_transform(self, transform, future_gap):
        """Compensate translation and rotation over a small TF time gap."""
        if not self.odom_history or future_gap <= 0.0:
            return transform

        transform_stamp = Time.from_msg(transform.header.stamp).nanoseconds
        odometry = min(
            self.odom_history,
            key=lambda item: abs(
                Time.from_msg(item.header.stamp).nanoseconds
                - transform_stamp
            )
        )

        camera_position = [
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ]
        camera_quaternion = [
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ]
        base_position = [
            odometry.pose.pose.position.x,
            odometry.pose.pose.position.y,
            odometry.pose.pose.position.z,
        ]
        base_quaternion = [
            odometry.pose.pose.orientation.x,
            odometry.pose.pose.orientation.y,
            odometry.pose.pose.orientation.z,
            odometry.pose.pose.orientation.w,
        ]
        body_velocity = [
            odometry.twist.twist.linear.x,
            odometry.twist.twist.linear.y,
            odometry.twist.twist.linear.z,
        ]

        position, quaternion = extrapolate_planar_camera_pose(
            camera_position,
            camera_quaternion,
            base_position,
            base_quaternion,
            body_velocity,
            odometry.twist.twist.angular.z,
            future_gap,
        )
        transform.transform.translation.x = float(position[0])
        transform.transform.translation.y = float(position[1])
        transform.transform.translation.z = float(position[2])
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])
        transform.header.stamp = (
            Time.from_msg(transform.header.stamp)
            + RclpyDuration(seconds=future_gap)
        ).to_msg()
        return transform

    def normalized_detection_time(self, stamp):

        source_time = Time.from_msg(stamp)
        arrival_time = self.get_clock().now()
        candidate_offset_ns = (
            arrival_time.nanoseconds
            - source_time.nanoseconds
        )

        if (
            abs(candidate_offset_ns) * 1e-9
            <= self.source_stamp_clock_tolerance
        ):
            return source_time

        self.source_time_offsets_ns.append(candidate_offset_ns)
        normalized_time_ns = (
            source_time.nanoseconds
            + min(self.source_time_offsets_ns)
        )

        normalized_time = Time(
            nanoseconds=normalized_time_ns,
            clock_type=arrival_time.clock_type
        )

        self.get_logger().warn(
            'Detection timestamps use a different clock; applying an '
            'arrival-derived clock offset for TF lookup',
            throttle_duration_sec=10.0
        )

        return normalized_time

    def detection_callback(
        self,
        msg
    ):

        # -------------------------------------------------------------
        # Keep YOLO time ONLY for the Kalman dt.
        # -------------------------------------------------------------

        detection_time = self.normalized_detection_time(
            msg.header.stamp
        )
        output_stamp = detection_time.to_msg()
        timestamp = detection_time.nanoseconds * 1e-9

        marker_array = MarkerArray()

        pose_array = PoseArray()

        brne_array = PedestrianArray()
        brne_array.header.stamp = output_stamp
        brne_array.header.frame_id = self.planning_frame

        planning_transform = None

        try:

            latest_transform = self.tf_buffer.lookup_transform(
                self.planning_frame,
                self.camera_frame,
                Time()
            )

            latest_time = Time.from_msg(
                latest_transform.header.stamp
            )
            future_gap = (
                detection_time
                - latest_time
            ).nanoseconds * 1e-9

            if future_gap > 0.0:

                if future_gap <= self.max_tf_future_tolerance:
                    planning_transform = self.extrapolate_latest_transform(
                        latest_transform,
                        future_gap
                    )

                else:
                    self.get_logger().warn(
                        f'Detection is {future_gap:.3f}s newer than robot '
                        f'TF; skipping frame',
                        throttle_duration_sec=3.0
                    )

            else:

                planning_transform = self.tf_buffer.lookup_transform(
                    self.planning_frame,
                    self.camera_frame,
                    detection_time,
                    timeout=RclpyDuration(
                        seconds=self.tf_lookup_timeout
                    )
                )

        except TransformException as exception:

            self.get_logger().warn(
                f'Cannot transform {self.camera_frame} to '
                f'{self.planning_frame} at detection time; '
                f'skipping frame to avoid false ego-motion: {exception}',
                throttle_duration_sec=3.0
            )

        pose_array.header.frame_id = (
            self.planning_frame
        )
        pose_array.header.stamp = output_stamp

        # =============================================================
        # PEOPLE
        # =============================================================

        for detection in msg.detections:

            if detection.class_name != 'person':
                continue

            pedestrian_id = detection.id

            if pedestrian_id == '':
                continue

            frame_id = (
                detection.bbox3d.frame_id
            )

            if frame_id != self.camera_frame:

                self.get_logger().warn(
                    f'Detection frame "{frame_id}". '
                    f'Expected {self.camera_frame}.',
                    throttle_duration_sec=3.0
                )

                continue

            # =========================================================
            # RAW YOLO CENTER
            # =========================================================

            measured_x = (
                detection.bbox3d
                .center.position.x
            )

            measured_y = (
                detection.bbox3d
                .center.position.y
            )

            measured_z = (
                detection.bbox3d
                .center.position.z
            )

            # Compensate ego motion before estimating pedestrian motion.
            # The YOLO measurement is camera-relative, while a stationary
            # pedestrian must remain stationary in the planning frame.
            if planning_transform is None:
                continue

            rotation = planning_transform.transform.rotation
            translation = planning_transform.transform.translation

            planning_position = self.rotate_vector(
                [measured_x, measured_y, measured_z],
                rotation
            )
            planning_position += np.array(
                [
                    translation.x,
                    translation.y,
                    translation.z,
                ]
            )

            # =========================================================
            # FILTER
            # =========================================================

            track = self.update_track(
                pedestrian_id,
                float(planning_position[0]),
                float(planning_position[1]),
                float(planning_position[2]),
                timestamp
            )

            if track['hits'] < self.minimum_track_hits:
                continue

            x = float(
                track['x'][0]
            )

            y = float(
                track['x'][1]
            )

            vx = float(
                track['x'][2]
            )

            vy = float(
                track['x'][3]
            )

            if math.hypot(vx, vy) < self.stationary_speed_threshold:
                vx = 0.0
                vy = 0.0

            z = float(
                track['z']
            )

            speed = math.hypot(
                vx,
                vy
            )

            heading = track[
                'heading'
            ]

            marker_id = (
                self.marker_id_from_track_id(
                    pedestrian_id
                )
            )

            # =========================================================
            # POSE
            # =========================================================

            pose = Pose()

            pose.position.x = x
            pose.position.y = y
            pose.position.z = z

            pose.orientation.z = math.sin(
                heading / 2.0
            )

            pose.orientation.w = math.cos(
                heading / 2.0
            )

            pose_array.poses.append(
                pose
            )

            # =========================================================
            # BRNE PEDESTRIAN (already in the planning frame)
            # =========================================================

            pedestrian = Pedestrian()
            pedestrian.header = brne_array.header
            pedestrian.id = self.brne_id_from_track_id(
                pedestrian_id
            )

            pedestrian.pose.position.x = x
            pedestrian.pose.position.y = y
            pedestrian.pose.position.z = z
            pedestrian.pose.orientation.z = math.sin(
                heading / 2.0
            )
            pedestrian.pose.orientation.w = math.cos(
                heading / 2.0
            )

            pedestrian.velocity.linear.x = vx
            pedestrian.velocity.linear.y = vy
            pedestrian.velocity.linear.z = 0.0

            brne_array.pedestrians.append(
                pedestrian
            )

            # =========================================================
            # POSITION SPHERE
            # =========================================================

            position_marker = Marker()

            position_marker.header.frame_id = (
                self.planning_frame
            )
            position_marker.header.stamp = output_stamp

            position_marker.ns = (
                'pedestrian_position'
            )

            position_marker.id = marker_id

            position_marker.type = (
                Marker.SPHERE
            )

            position_marker.action = (
                Marker.ADD
            )

            position_marker.pose.position.x = x
            position_marker.pose.position.y = y
            position_marker.pose.position.z = z

            position_marker.pose.orientation.w = 1.0

            position_marker.scale.x = 0.15
            position_marker.scale.y = 0.15
            position_marker.scale.z = 0.15

            position_marker.color.r = 0.1
            position_marker.color.g = 1.0
            position_marker.color.b = 0.1
            position_marker.color.a = 1.0

            position_marker.lifetime = (
                self.marker_lifetime
            )

            marker_array.markers.append(
                position_marker
            )

            # =========================================================
            # VELOCITY ARROW
            # =========================================================

            if track['arrow_visible']:

                velocity_marker = Marker()

                velocity_marker.header.frame_id = (
                    self.planning_frame
                )
                velocity_marker.header.stamp = output_stamp

                velocity_marker.ns = (
                    'pedestrian_velocity'
                )

                velocity_marker.id = marker_id

                velocity_marker.type = (
                    Marker.ARROW
                )

                velocity_marker.action = (
                    Marker.ADD
                )

                start = Point()

                start.x = x
                start.y = y
                start.z = z

                # Protect against divide-by-zero.
                if speed > 1e-6:

                    direction_x = (
                        vx / speed
                    )

                    direction_y = (
                        vy / speed
                    )

                else:

                    direction_x = 0.0
                    direction_y = 0.0

                arrow_length = min(
                    self.arrow_scale
                    * speed,
                    self.maximum_arrow_length
                )

                end = Point()

                end.x = (
                    x
                    + direction_x
                    * arrow_length
                )

                end.y = (
                    y
                    + direction_y
                    * arrow_length
                )

                end.z = z

                velocity_marker.points = [
                    start,
                    end
                ]

                velocity_marker.scale.x = 0.04
                velocity_marker.scale.y = 0.10
                velocity_marker.scale.z = 0.12

                velocity_marker.color.r = 1.0
                velocity_marker.color.g = 0.1
                velocity_marker.color.b = 0.1
                velocity_marker.color.a = 1.0

                velocity_marker.lifetime = (
                    self.marker_lifetime
                )

                marker_array.markers.append(
                    velocity_marker
                )

            # =========================================================
            # TEXT
            # =========================================================

            text_marker = Marker()

            text_marker.header.frame_id = (
                self.planning_frame
            )
            text_marker.header.stamp = output_stamp

            text_marker.ns = (
                'pedestrian_text'
            )

            text_marker.id = marker_id

            text_marker.type = (
                Marker.TEXT_VIEW_FACING
            )

            text_marker.action = (
                Marker.ADD
            )

            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            text_marker.pose.position.z = (
                z + 0.30
            )

            text_marker.pose.orientation.w = 1.0

            text_marker.scale.z = 0.15

            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            text_marker.text = (
                f'ID {pedestrian_id}\n'
                f'{speed:.2f} m/s'
            )

            text_marker.lifetime = (
                self.marker_lifetime
            )

            marker_array.markers.append(
                text_marker
            )

            # =========================================================
            # LOG
            # =========================================================

            self.get_logger().info(
                f'ID={pedestrian_id} | '
                f'P=({x:.2f},{y:.2f},{z:.2f}) | '
                f'V=({vx:.2f},{vy:.2f}) | '
                f'speed={speed:.2f} | '
                f'arrow={track["arrow_visible"]}'
            )

        # =============================================================
        # IMPORTANT:
        #
        # DO NOT delete pedestrian markers immediately when one YOLO
        # frame misses the person.
        #
        # Marker lifetime will remove them naturally.
        # =============================================================

        # =============================================================
        # DELETE STALE INTERNAL TRACKS
        # =============================================================

        stale_ids = []

        for pedestrian_id, track in self.tracks.items():

            age = (
                timestamp
                - track['last_seen']
            )

            if age > self.track_timeout:

                stale_ids.append(
                    pedestrian_id
                )

        for pedestrian_id in stale_ids:

            del self.tracks[
                pedestrian_id
            ]

            brne_id = self.brne_ids.pop(
                pedestrian_id,
                None
            )

            if brne_id is not None:

                self.free_brne_ids.append(brne_id)
                self.free_brne_ids.sort()

        # =============================================================
        # PUBLISH
        # =============================================================

        self.marker_pub.publish(
            marker_array
        )

        self.pose_pub.publish(
            pose_array
        )

        self.brne_pedestrian_pub.publish(
            brne_array
        )


# =====================================================================
# MAIN
# =====================================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = PedestrianStateNode()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()

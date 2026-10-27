#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import (
    PoseStamped,
    TransformStamped,
)

from nav_msgs.msg import Odometry
from unitree_go.msg import SportModeState

from tf2_ros import TransformBroadcaster


class RobotPoseTF(Node):

    def __init__(self):

        super().__init__('robot_pose_tf')

        # =============================================================
        # Parameters
        # =============================================================

        self.declare_parameter(
            'pose_topic',
            '/utlidar/robot_pose'
        )

        self.declare_parameter(
            'sport_state_topic',
            '/lf/sportmodestate'
        )

        self.declare_parameter(
            'use_sport_state',
            True
        )

        self.declare_parameter(
            'pose_timeout',
            0.5
        )

        self.declare_parameter(
            'parent_frame',
            'brne_odom'
        )

        self.declare_parameter(
            'child_frame',
            'base_link'
        )

        self.declare_parameter(
            'odom_topic',
            '/brne_odom'
        )

        # =============================================================
        # Read parameters
        # =============================================================

        self.pose_topic = self.get_parameter(
            'pose_topic'
        ).value

        self.sport_state_topic = self.get_parameter(
            'sport_state_topic'
        ).value

        self.use_sport_state = bool(
            self.get_parameter(
                'use_sport_state'
            ).value
        )

        self.pose_timeout = float(
            self.get_parameter(
                'pose_timeout'
            ).value
        )

        self.parent_frame = self.get_parameter(
            'parent_frame'
        ).value

        self.child_frame = self.get_parameter(
            'child_frame'
        ).value

        self.odom_topic = self.get_parameter(
            'odom_topic'
        ).value

        # =============================================================
        # TF broadcaster
        # =============================================================

        self.tf_broadcaster = (
            TransformBroadcaster(self)
        )

        # =============================================================
        # Robot pose subscriber
        # =============================================================

        if self.use_sport_state:

            self.subscription = self.create_subscription(
                SportModeState,
                self.sport_state_topic,
                self.sport_state_callback,
                qos_profile_sensor_data
            )

        else:

            self.subscription = self.create_subscription(
                PoseStamped,
                self.pose_topic,
                self.pose_callback,
                qos_profile_sensor_data
            )

        self.odom_publisher = self.create_publisher(
            Odometry,
            self.odom_topic,
            20
        )

        self.last_pose_time = None
        self.watchdog_timer = self.create_timer(
            0.5,
            self.pose_watchdog
        )

        # =============================================================
        # Information
        # =============================================================

        source_topic = (
            self.sport_state_topic
            if self.use_sport_state
            else self.pose_topic
        )

        self.get_logger().info(
            f'Listening to continuous robot pose: {source_topic}'
        )

        self.get_logger().info(
            f'Publishing dynamic TF: '
            f'{self.parent_frame} -> {self.child_frame}'
        )

        self.get_logger().info(
            f'Publishing BRNE odometry: {self.odom_topic}'
        )

        self.get_logger().info(
            'TF and odometry timestamps use current ROS time'
        )

    # =================================================================
    # Robot pose callback
    # =================================================================

    def pose_callback(self, msg):

        self.publish_pose(
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )

    def sport_state_callback(self, msg):

        quaternion = msg.imu_state.quaternion

        # Unitree SportModeState stores quaternion as [w, x, y, z].
        self.publish_pose(
            float(msg.position[0]),
            float(msg.position[1]),
            float(msg.position[2]),
            float(quaternion[1]),
            float(quaternion[2]),
            float(quaternion[3]),
            float(quaternion[0]),
            velocity=msg.velocity,
            yaw_speed=float(msg.yaw_speed),
        )

    def publish_pose(
        self,
        x,
        y,
        z,
        qx,
        qy,
        qz,
        qw,
        velocity=None,
        yaw_speed=0.0,
    ):

        transform = TransformStamped()

        # =============================================================
        # IMPORTANT
        #
        # Do NOT use:
        #
        # transform.header.stamp = msg.header.stamp
        #
        # because Unitree source stamps and YOLO currently use different
        # timestamp bases.
        #
        # Use current ROS time so RViz can resolve:
        #
        # odom -> base_link -> camera_link
        # =============================================================

        transform.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        transform.header.frame_id = (
            self.parent_frame
        )

        transform.child_frame_id = (
            self.child_frame
        )

        # =============================================================
        # Translation
        # =============================================================

        transform.transform.translation.x = float(x)
        transform.transform.translation.y = float(y)
        transform.transform.translation.z = float(z)

        # =============================================================
        # Rotation
        # =============================================================

        transform.transform.rotation.x = float(qx)
        transform.transform.rotation.y = float(qy)
        transform.transform.rotation.z = float(qz)
        transform.transform.rotation.w = float(qw)

        # =============================================================
        # Publish TF
        # =============================================================

        self.tf_broadcaster.sendTransform(
            transform
        )

        # BRNE consumes nav_msgs/Odometry. The Unitree state position is
        # already expressed in its odometry frame, so preserve its numeric
        # pose while using the same planning frame as the TF tree.
        odometry = Odometry()
        odometry.header.stamp = transform.header.stamp
        odometry.header.frame_id = self.parent_frame
        odometry.child_frame_id = self.child_frame
        odometry.pose.pose.position.x = float(x)
        odometry.pose.pose.position.y = float(y)
        odometry.pose.pose.position.z = float(z)
        odometry.pose.pose.orientation.x = float(qx)
        odometry.pose.pose.orientation.y = float(qy)
        odometry.pose.pose.orientation.z = float(qz)
        odometry.pose.pose.orientation.w = float(qw)

        if velocity is not None:
            odometry.twist.twist.linear.x = float(velocity[0])
            odometry.twist.twist.linear.y = float(velocity[1])
            odometry.twist.twist.linear.z = float(velocity[2])
            odometry.twist.twist.angular.z = float(yaw_speed)

        self.odom_publisher.publish(
            odometry
        )

        self.last_pose_time = self.get_clock().now()

    def pose_watchdog(self):

        if self.last_pose_time is None:
            self.get_logger().warn(
                'No robot pose received; ego-motion compensation is '
                'unavailable',
                throttle_duration_sec=3.0
            )
            return

        age = (
            self.get_clock().now()
            - self.last_pose_time
        ).nanoseconds * 1e-9

        if age > self.pose_timeout:
            self.get_logger().warn(
                f'Robot pose is stale ({age:.2f}s); pedestrian frames '
                f'will be skipped until TF recovers',
                throttle_duration_sec=3.0
            )


# =====================================================================
# Main
# =====================================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = RobotPoseTF()

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

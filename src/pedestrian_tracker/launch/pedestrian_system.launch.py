#!/usr/bin/env python3

import os

from launch import LaunchDescription

from launch_ros.actions import Node

from ament_index_python.packages import (
    get_package_share_directory
)


def generate_launch_description():

    package_share = get_package_share_directory(
        'pedestrian_tracker'
    )

    rviz_config = os.path.join(
        package_share,
        'config',
        'pedestrian.rviz'
    )

    # =============================================================
    # odom -> base_link
    # From /utlidar/robot_pose
    # =============================================================

    robot_pose_tf_node = Node(
        package='pedestrian_tracker',
        executable='robot_pose_tf',
        name='robot_pose_tf',
        output='screen'
    )

    # =============================================================
    # Pedestrian tracking
    #
    # Also publishes:
    # base_link -> camera_link
    #
    # and:
    # /pedestrians/markers
    # /pedestrians/poses
    # =============================================================

    pedestrian_state_node = Node(
        package='pedestrian_tracker',
        executable='pedestrian_state_node',
        name='pedestrian_state_node',
        output='screen'
    )

    # =============================================================
    # Green marker exactly at camera_link
    # =============================================================

    camera_marker_node = Node(
        package='pedestrian_tracker',
        executable='camera_link_marker',
        name='camera_link_marker',
        output='screen'
    )

    # =============================================================
    # RViz
    # =============================================================

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            rviz_config
        ]
    )

    return LaunchDescription([
        robot_pose_tf_node,
        pedestrian_state_node,
        camera_marker_node,
        rviz_node,
    ])

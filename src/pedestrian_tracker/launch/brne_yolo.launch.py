#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    crowd_nav_share = get_package_share_directory('crowd_nav')
    brne_config = os.path.join(
        crowd_nav_share,
        'config',
        'brne.yaml'
    )
    rviz_config = os.path.join(
        crowd_nav_share,
        'config',
        'crowdnav.rviz'
    )

    arguments = [
        DeclareLaunchArgument(
            'detections_topic',
            default_value='/yolo/detections_3d'
        ),
        DeclareLaunchArgument(
            'robot_pose_topic',
            default_value='/utlidar/robot_pose'
        ),
        DeclareLaunchArgument(
            'sport_state_topic',
            default_value='/lf/sportmodestate',
            description='Continuous 20 Hz Unitree odometry/state source.'
        ),
        DeclareLaunchArgument(
            'use_sport_state_pose',
            default_value='true',
            description=(
                'Use SportModeState pose instead of the slow lidar pose.'
            )
        ),
        DeclareLaunchArgument(
            'pedestrians_topic',
            default_value='/pedestrians'
        ),
        DeclareLaunchArgument(
            'pedestrian_markers_topic',
            default_value='/ped_markers'
        ),
        DeclareLaunchArgument(
            'brne_odom_topic',
            default_value='/brne_odom'
        ),
        DeclareLaunchArgument(
            'goal_topic',
            default_value='/goal_pose'
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel'
        ),
        DeclareLaunchArgument(
            'planning_frame',
            default_value='brne_odom'
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link'
        ),
        DeclareLaunchArgument(
            'camera_frame',
            default_value='camera_link'
        ),
        DeclareLaunchArgument(
            'camera_x',
            default_value='0.34'
        ),
        DeclareLaunchArgument(
            'camera_y',
            default_value='0.0'
        ),
        DeclareLaunchArgument(
            'camera_z',
            default_value='0.08'
        ),
        DeclareLaunchArgument(
            'camera_upward_pitch_deg',
            default_value='30.0',
            description=(
                'Physical upward tilt of camera_link in degrees.'
            )
        ),
        DeclareLaunchArgument(
            'tf_lookup_timeout',
            default_value='0.08',
            description=(
                'TF lookup timeout at the YOLO image timestamp.'
            )
        ),
        DeclareLaunchArgument(
            'max_tf_future_tolerance',
            default_value='0.075',
            description=(
                'Maximum small future gap allowed against the latest 20 Hz TF.'
            )
        ),
        DeclareLaunchArgument(
            'source_stamp_clock_tolerance',
            default_value='5.0',
            description=(
                'Normalize camera stamps when their clock differs from '
                'ROS time.'
            )
        ),
        DeclareLaunchArgument(
            'z_smoothing',
            default_value='0.0',
            description=(
                'Previous-Z weight; zero keeps the green center aligned '
                'with the current YOLO 3D box.'
            )
        ),
        DeclareLaunchArgument(
            'offset_unitree_vel',
            default_value='false'
        ),
        DeclareLaunchArgument(
            'use_walls',
            default_value='false',
            description=(
                'Display the legacy corridor wall markers.'
            )
        ),
        DeclareLaunchArgument(
            'use_corridor_mask',
            default_value='false',
            description='Reject BRNE samples outside legacy corridor y-limits.'
        ),
        DeclareLaunchArgument(
            'publish_robot_samples',
            default_value='true'
        ),
        DeclareLaunchArgument(
            'publish_pedestrian_samples',
            default_value='true'
        ),
        DeclareLaunchArgument(
            'robot_sample_alpha',
            default_value='0.20'
        ),
        DeclareLaunchArgument(
            'pedestrian_sample_alpha',
            default_value='0.18'
        ),
        DeclareLaunchArgument(
            'heading_gain',
            default_value='1.0',
            description=(
                'Fixed goal-heading gain: omega = gain * heading error.'
            )
        ),
        DeclareLaunchArgument(
            'start_controller',
            default_value='false',
            description=(
                'Publish BRNE commands to cmd_vel. Leave false while '
                'validating perception and frames.'
            )
        ),
        DeclareLaunchArgument(
            'start_unitree_bridge',
            default_value='false',
            description='Start the cmd_vel to Unitree Go2 Sport API bridge.'
        ),
        DeclareLaunchArgument(
            'unitree_bridge_enabled',
            default_value='false',
            description='Arm physical Go2 motion commands at startup.'
        ),
        DeclareLaunchArgument(
            'unitree_auto_arm_on_goal',
            default_value='true',
            description='Automatically arm safe Go2 motion on each new goal.'
        ),
        DeclareLaunchArgument(
            'unitree_max_linear_x',
            default_value='0.20'
        ),
        DeclareLaunchArgument(
            'unitree_max_angular_z',
            default_value='0.40'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false'
        ),
    ]

    robot_pose_tf = Node(
        package='pedestrian_tracker',
        executable='robot_pose_tf',
        name='robot_pose_tf',
        output='screen',
        parameters=[{
            'pose_topic': LaunchConfiguration('robot_pose_topic'),
            'sport_state_topic': LaunchConfiguration('sport_state_topic'),
            'use_sport_state': ParameterValue(
                LaunchConfiguration('use_sport_state_pose'),
                value_type=bool
            ),
            'parent_frame': LaunchConfiguration('planning_frame'),
            'child_frame': LaunchConfiguration('base_frame'),
            'odom_topic': LaunchConfiguration('brne_odom_topic'),
        }]
    )

    pedestrian_state = Node(
        package='pedestrian_tracker',
        executable='pedestrian_state_node',
        name='pedestrian_state_node',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('detections_topic'),
            'brne_pedestrian_topic': LaunchConfiguration(
                'pedestrians_topic'
            ),
            'planning_frame': LaunchConfiguration('planning_frame'),
            'odom_topic': LaunchConfiguration('brne_odom_topic'),
            'camera_parent_frame': LaunchConfiguration('base_frame'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'camera_x': ParameterValue(
                LaunchConfiguration('camera_x'),
                value_type=float
            ),
            'camera_y': ParameterValue(
                LaunchConfiguration('camera_y'),
                value_type=float
            ),
            'camera_z': ParameterValue(
                LaunchConfiguration('camera_z'),
                value_type=float
            ),
            'camera_upward_pitch_deg': ParameterValue(
                LaunchConfiguration('camera_upward_pitch_deg'),
                value_type=float
            ),
            'tf_lookup_timeout': ParameterValue(
                LaunchConfiguration('tf_lookup_timeout'),
                value_type=float
            ),
            'max_tf_future_tolerance': ParameterValue(
                LaunchConfiguration('max_tf_future_tolerance'),
                value_type=float
            ),
            'source_stamp_clock_tolerance': ParameterValue(
                LaunchConfiguration('source_stamp_clock_tolerance'),
                value_type=float
            ),
            'z_smoothing': ParameterValue(
                LaunchConfiguration('z_smoothing'),
                value_type=float
            ),
        }],
        remappings=[
            (
                '/pedestrians/markers',
                LaunchConfiguration('pedestrian_markers_topic')
            ),
        ]
    )

    brne = Node(
        package='crowd_nav',
        executable='brne_nav',
        name='brne',
        output='screen',
        # BRNE parallelizes its pairwise cost matrix with OpenMP.  On the
        # CPU-only perception stack, leaving the runtime unconstrained can
        # exhaust the process/thread limit and libgomp aborts the node.  This
        # only bounds worker concurrency; it does not change any planner
        # samples, costs, weights, masks, or commands.
        additional_env={
            'OMP_NUM_THREADS': '4',
            'OMP_THREAD_LIMIT': '4',
        },
        parameters=[
            brne_config,
            {
                'offset_unitree_vel': ParameterValue(
                    LaunchConfiguration('offset_unitree_vel'),
                    value_type=bool
                ),
                'use_walls': ParameterValue(
                    LaunchConfiguration('use_walls'),
                    value_type=bool
                ),
                'use_corridor_mask': ParameterValue(
                    LaunchConfiguration('use_corridor_mask'),
                    value_type=bool
                ),
                'planning_frame': LaunchConfiguration('planning_frame'),
                'publish_robot_samples': ParameterValue(
                    LaunchConfiguration('publish_robot_samples'),
                    value_type=bool
                ),
                'publish_pedestrian_samples': ParameterValue(
                    LaunchConfiguration('publish_pedestrian_samples'),
                    value_type=bool
                ),
                'robot_sample_alpha': ParameterValue(
                    LaunchConfiguration('robot_sample_alpha'),
                    value_type=float
                ),
                'pedestrian_sample_alpha': ParameterValue(
                    LaunchConfiguration('pedestrian_sample_alpha'),
                    value_type=float
                ),
                'heading_gain': ParameterValue(
                    LaunchConfiguration('heading_gain'),
                    value_type=float
                ),
            },
        ],
        remappings=[
            ('pedestrians', LaunchConfiguration('pedestrians_topic')),
            ('brne_odom', LaunchConfiguration('brne_odom_topic')),
            ('goal_pose', LaunchConfiguration('goal_topic')),
        ]
    )

    controller = Node(
        package='crowd_nav',
        executable='controller',
        name='controller',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_controller')),
        parameters=[brne_config],
        remappings=[
            ('cmd_vel', LaunchConfiguration('cmd_vel_topic')),
        ]
    )

    unitree_bridge = Node(
        package='pedestrian_tracker',
        executable='unitree_cmd_vel_bridge',
        name='unitree_cmd_vel_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_unitree_bridge')),
        parameters=[{
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'goal_topic': LaunchConfiguration('goal_topic'),
            'enabled': ParameterValue(
                LaunchConfiguration('unitree_bridge_enabled'),
                value_type=bool
            ),
            'auto_arm_on_goal': ParameterValue(
                LaunchConfiguration('unitree_auto_arm_on_goal'),
                value_type=bool
            ),
            'max_linear_x': ParameterValue(
                LaunchConfiguration('unitree_max_linear_x'),
                value_type=float
            ),
            'max_angular_z': ParameterValue(
                LaunchConfiguration('unitree_max_angular_z'),
                value_type=float
            ),
        }]
    )

    goal_service = Node(
        package='crowd_nav',
        executable='pub_goal',
        name='pub_goal',
        output='screen',
        remappings=[
            ('goal_pose', LaunchConfiguration('goal_topic')),
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', rviz_config]
    )

    return LaunchDescription(
        arguments + [
            robot_pose_tf,
            pedestrian_state,
            brne,
            controller,
            unitree_bridge,
            goal_service,
            rviz,
        ]
    )

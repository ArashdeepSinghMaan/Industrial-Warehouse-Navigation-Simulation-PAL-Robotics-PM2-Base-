#!/usr/bin/env python3
"""
Minimal RTAB-Map launch for 2D LiDAR only (no cameras, no odometry input).
Place this file into your_package/launch/lidar_only_launch.py
and create a corresponding YAML at your_package/config/lidar_only_params.yaml
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Path to external params YAML
    config_file = os.path.join(
        get_package_share_directory('warehouse_robot'),
        'config',
        'lidar_only_params.yaml'
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock'
        ),
        DeclareLaunchArgument(
            'scan_topic', default_value='/lidar',
            description='Input topic for 2D LiDAR scans'
        ),
        DeclareLaunchArgument(
            'map_frame_id', default_value='map',
            description='Output map frame id'
        ),

        # RTAB-Map SLAM Node (LiDAR only)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap_lidar_only',
            output='screen',
            parameters=[
                config_file,
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'scan_topic': LaunchConfiguration('scan_topic'),
                    'frame_id': 'base_link',
                    'map_frame_id': LaunchConfiguration('map_frame_id'),
                }
            ],
            remappings=[
                ('scan', LaunchConfiguration('scan_topic')),
                ('map',  '/map')
            ],
        ),
    ])

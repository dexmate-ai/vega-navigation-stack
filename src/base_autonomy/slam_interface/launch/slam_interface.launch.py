#!/usr/bin/env python3
"""
Launch SLAM interface with YAML configuration files.
This node publishes state_estimation, preprocesses pointclouds, and manages static transforms.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('slam_interface')
    
    # Default config file paths
    default_transform_config = os.path.join(pkg_dir, 'config', 'calibration', 'transform_default.yaml')
    default_filter_config = os.path.join(pkg_dir, 'config', 'body_filter_default.yaml')
    
    # Declare launch arguments
    transform_config_arg = DeclareLaunchArgument(
        'transform_config',
        default_value=default_transform_config,
        description='Path to transform calibration YAML file (defines map<->init_frame and vehicle<->laser transforms)'
    )
    
    filter_config_arg = DeclareLaunchArgument(
        'filter_config',
        default_value=default_filter_config,
        description='Path to body filter configuration YAML file (defines pointcloud filtering box)'
    )
    
    # Get launch configurations
    transform_config = LaunchConfiguration('transform_config')
    filter_config = LaunchConfiguration('filter_config')
    
    # Create SLAM interface node with config files
    slam_interface_node = Node(
        package='slam_interface',
        executable='slam_interface',
        name='slam_interface',
        output='screen',
        parameters=[
            transform_config,
            filter_config
        ]
    )
    
    return LaunchDescription([
        transform_config_arg,
        filter_config_arg,
        slam_interface_node
    ])


"""
Real robot system launch file for RS Airy LiDAR configuration.
Includes base components + RS Airy specific sensor/SLAM setup + RViz.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for RS Airy real robot system."""

    # Launch arguments
    global_map_path = LaunchConfiguration('global_map_path')

    declare_global_map_path = DeclareLaunchArgument(
        'global_map_path',
        default_value="/workspace/src/slam/super_odom/maps_internal/map_lidar_init.pcd",
        description='Path to the global map PCD file for background terrain'
    )

    # # Include base real robot components
    # include_base = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(
    #         get_package_share_directory('vehicle_simulator'),
    #         'launch',
    #         'system_real_robot_base.launch.py'
    #     )),
    #     launch_arguments={
    #         'global_map_path': global_map_path,
    #         'use_prior_map': 'true',  # Explicitly enable prior map for localization
    #     }.items()
    # )

    include_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('vehicle_simulator'),
            'launch',
            'system_real_robot_base.launch.py'
        )),
    )

    # RS Airy specific: LiDAR driver
    start_airy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('rslidar_sdk'), '/launch/start.py']
        ),
    )

    # RS Airy specific: SLAM/Odometry
    start_super_odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('super_odometry'), 'launch', 'rs_airy_localization.launch.py')
        ),
        launch_arguments={
            'prior_map': global_map_path,
        }.items()
    )

    # RS Airy specific: SLAM interface with RS Airy transform config
    start_slam_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_interface'), 'launch', 'slam_interface.launch.py')
        ),
        launch_arguments={
            'transform_config': os.path.join(
                get_package_share_directory('slam_interface'),
                'config', 'calibration', 'transform_airy_v1p.yaml'
            ),
            'filter_config': os.path.join(
                get_package_share_directory('slam_interface'),
                'config', 'body_filter_default.yaml'
            )
        }.items()
    )

    # RViz with 3D configuration
    start_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            os.path.join(
                get_package_share_directory('vehicle_simulator'),
                'rviz',
                'vehicle_simulator_3d.rviz'
            )
        ],
        parameters=[{'use_sim_time': False}],
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_global_map_path)

    # Add all components
    ld.add_action(include_base)
    ld.add_action(start_airy)
    ld.add_action(start_super_odometry)
    ld.add_action(start_slam_interface)
    ld.add_action(start_rviz2)

    return ld

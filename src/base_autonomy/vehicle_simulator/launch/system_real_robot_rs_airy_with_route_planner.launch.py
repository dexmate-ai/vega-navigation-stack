"""
Real robot system launch file for RS Airy LiDAR configuration.
Includes base components + RS Airy specific sensor/SLAM setup + RViz.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for RS Airy real robot system."""

    route_planner_config = LaunchConfiguration('route_planner_config')

     # Declare route planner config argument
    declare_route_planner_config = DeclareLaunchArgument(
        'route_planner_config',
        default_value='indoor',
        description='Route planner configuration'
    )

    # Include base real robot components
    include_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('vehicle_simulator'),
            'launch',
            'system_real_robot_base.launch.py'
        ))
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
            get_package_share_directory('super_odometry'), 'launch', 'rs_airy.launch.py')
        )
    )

    # RS Airy specific: SLAM interface with RS Airy transform config
    start_slam_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_interface'), 'launch', 'slam_interface.launch.py')
        ),
        launch_arguments={
            'transform_config': os.path.join(
                get_package_share_directory('slam_interface'),
                'config', 'calibration', 'transform_airy_zm90_xm40.yaml'
            ),
            'filter_config': os.path.join(
                get_package_share_directory('slam_interface'),
                'config', 'body_filter_default.yaml'
            )
        }.items()
    )

    # RViz with FAR Planner configuration
    start_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            os.path.join(
                get_package_share_directory('far_planner'),
                'rviz',
                'default.rviz'
            )
        ],
        parameters=[{'use_sim_time': False}],
    )

    start_far_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('far_planner'), '/launch/far_planner.launch']
        ),
        launch_arguments={
            'config': route_planner_config,
        }.items()
    )

    # Create launch description
    ld = LaunchDescription()

    # Add all components
    ld.add_action(declare_route_planner_config)
    ld.add_action(include_base)
    ld.add_action(start_airy)
    ld.add_action(start_super_odometry)
    ld.add_action(start_slam_interface)
    ld.add_action(start_far_planner)
    ld.add_action(start_rviz2)

    return ld

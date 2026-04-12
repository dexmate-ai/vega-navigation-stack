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
    slam_calibration_config = LaunchConfiguration('slam_calibration_config')
    local_planner_config = LaunchConfiguration('local_planner_config')

    # Declare launch arguments
    declare_slam_calibration_config = DeclareLaunchArgument(
        'slam_calibration_config',
        default_value='transform_airy_v1p_omni',
        description='SLAM interface calibration config file name (without .yaml extension)'
    )
    declare_local_planner_config = DeclareLaunchArgument(
        'local_planner_config',
        default_value='standard_3d_omni',
        description='Local planner configuration (standard_3d, standard_3d_omni, etc.)'
    )

    # Include base real robot components
    include_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('vehicle_simulator'),
            'launch',
            'system_real_robot_base.launch.py'
        )),
        launch_arguments={
            'local_planner_config': local_planner_config,
        }.items()
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
            'transform_config': [
                get_package_share_directory('slam_interface'),
                '/config/calibration/', slam_calibration_config, '.yaml'
            ],
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

    # Add launch argument declarations
    ld.add_action(declare_slam_calibration_config)
    ld.add_action(declare_local_planner_config)

    # Add all components
    ld.add_action(include_base)
    ld.add_action(start_airy)
    ld.add_action(start_super_odometry)
    ld.add_action(start_slam_interface)
    ld.add_action(start_rviz2)

    return ld

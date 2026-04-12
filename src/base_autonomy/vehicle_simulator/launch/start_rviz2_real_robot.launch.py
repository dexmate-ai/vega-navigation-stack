"""
Real robot system launch file for RS Airy LiDAR configuration.
Includes base components + RS Airy specific sensor/SLAM setup + RViz.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for RS Airy real robot system."""\

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

    ld.add_action(start_rviz2)

    return ld

"""
Base launch file for real robot systems.
This file contains common components shared across different real robot configurations.
Specific launch files should include this base and add their sensor/SLAM-specific components + RViz.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    """Generate base launch description with common components for real robot systems."""

    # Launch arguments
    world_name = LaunchConfiguration('world_name')
    sensorOffsetX = LaunchConfiguration('sensorOffsetX')
    sensorOffsetY = LaunchConfiguration('sensorOffsetY')
    cameraOffsetZ = LaunchConfiguration('cameraOffsetZ')
    vehicleX = LaunchConfiguration('vehicleX')
    vehicleY = LaunchConfiguration('vehicleY')
    checkTerrainConn = LaunchConfiguration('checkTerrainConn')
    local_planner_config = LaunchConfiguration('local_planner_config')
    controller_type = LaunchConfiguration('controller_type')
    joy_dev = LaunchConfiguration('joy_dev')
    joy_deadzone = LaunchConfiguration('joy_deadzone')
    global_map_path = LaunchConfiguration('global_map_path')
    use_prior_map = LaunchConfiguration('use_prior_map')

    # Declare common launch arguments
    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='real_world',
        description='World name for visualization'
    )
    declare_sensorOffsetX = DeclareLaunchArgument(
        'sensorOffsetX',
        default_value='0.05',
        description='Sensor offset in X direction'
    )
    declare_sensorOffsetY = DeclareLaunchArgument(
        'sensorOffsetY',
        default_value='0.0',
        description='Sensor offset in Y direction'
    )
    declare_cameraOffsetZ = DeclareLaunchArgument(
        'cameraOffsetZ',
        default_value='0.25',
        description='Camera offset in Z direction'
    )
    declare_vehicleX = DeclareLaunchArgument(
        'vehicleX',
        default_value='0.0',
        description='Vehicle initial X position'
    )
    declare_vehicleY = DeclareLaunchArgument(
        'vehicleY',
        default_value='0.0',
        description='Vehicle initial Y position'
    )
    declare_checkTerrainConn = DeclareLaunchArgument(
        'checkTerrainConn',
        default_value='true',
        description='Enable terrain connectivity check'
    )
    declare_local_planner_config = DeclareLaunchArgument(
        'local_planner_config',
        default_value='standard_3d',
        description='Local planner configuration (standard_3d, standard_2d, etc.)'
    )
    declare_controller_type = DeclareLaunchArgument(
        'controller_type',
        default_value='8bitdo',
        description='Controller type for joy_command_mapper'
    )
    declare_joy_dev = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )
    declare_joy_deadzone = DeclareLaunchArgument(
        'joy_deadzone',
        default_value='0.12',
        description='Joystick deadzone'
    )
    declare_global_map_path = DeclareLaunchArgument(
        'global_map_path',
        default_value='',
        description='Path to the global map PCD file for background terrain'
    )
    declare_use_prior_map = DeclareLaunchArgument(
        'use_prior_map',
        default_value='false',
        description='Enable prior map processing in terrain analysis'
    )

    # Common components
    start_local_planner = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(
            get_package_share_directory('local_planner'), 'launch', 'local_planner.launch')
        ),
        launch_arguments={
            'realRobot': 'true',
            'sensorOffsetX': sensorOffsetX,
            'sensorOffsetY': sensorOffsetY,
            'cameraOffsetZ': cameraOffsetZ,
            'goalX': vehicleX,
            'goalY': vehicleY,
        }.items()
    )

    start_terrain_analysis = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(
            get_package_share_directory('terrain_analysis'), 'launch', 'terrain_analysis.launch')
        ),
        launch_arguments={
            'global_map_path': global_map_path,
            'use_prior_map': use_prior_map,
        }.items()
    )

    start_terrain_analysis_ext = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(
            get_package_share_directory('terrain_analysis_ext'), 'launch', 'terrain_analysis_ext.launch')
        ),
        launch_arguments={
            'checkTerrainConn': checkTerrainConn,
            'local_planner_config': local_planner_config,
        }.items()
    )

    start_visualization_tools = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(
            get_package_share_directory('visualization_tools'), 'launch', 'visualization_tools.launch')
        ),
        launch_arguments={
            'world_name': world_name,
        }.items()
    )

    start_joy_command_mapper = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(
            get_package_share_directory('joy_command_mapper'), 'launch', 'joy_command_mapper.launch')
        ),
        launch_arguments={
            'controller_type': controller_type,
            'joy_dev': joy_dev,
            'joy_deadzone': joy_deadzone,
        }.items()
    )

    # Create launch description
    ld = LaunchDescription()

    # Add common launch arguments
    ld.add_action(declare_world_name)
    ld.add_action(declare_sensorOffsetX)
    ld.add_action(declare_sensorOffsetY)
    ld.add_action(declare_cameraOffsetZ)
    ld.add_action(declare_vehicleX)
    ld.add_action(declare_vehicleY)
    ld.add_action(declare_checkTerrainConn)
    ld.add_action(declare_local_planner_config)
    ld.add_action(declare_controller_type)
    ld.add_action(declare_joy_dev)
    ld.add_action(declare_joy_deadzone)
    ld.add_action(declare_global_map_path)
    ld.add_action(declare_use_prior_map)

    # Add common components
    ld.add_action(start_local_planner)
    ld.add_action(start_terrain_analysis)
    ld.add_action(start_terrain_analysis_ext)
    ld.add_action(start_visualization_tools)
    ld.add_action(start_joy_command_mapper)

    return ld

#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'goal_tolerance',
            default_value='0.25',
            description='Distance tolerance to consider waypoint reached (meters)'
        ),
        
        DeclareLaunchArgument(
            'sequence_topic',
            default_value='/waypoint_sequence',
            description='Topic for waypoint sequence data'
        ),
        
        DeclareLaunchArgument(
            'new_sequence_topic', 
            default_value='/new_waypoint_sequence',
            description='Topic for receiving new waypoint sequences'
        ),
        
        DeclareLaunchArgument(
            'sequence_control_topic',
            default_value='/waypoint_sequence_control',
            description='Topic for sequence control commands'
        ),
        
        Node(
            package='waypoint_sequence_manager',
            executable='waypoint_sequence_manager',
            name='waypoint_sequence_manager',
            output='screen',
            parameters=[{
                'goal_tolerance': LaunchConfiguration('goal_tolerance'),
                'sequence_topic': LaunchConfiguration('sequence_topic'),
                'new_sequence_topic': LaunchConfiguration('new_sequence_topic'),
                'sequence_control_topic': LaunchConfiguration('sequence_control_topic'),
            }]
        )
    ])
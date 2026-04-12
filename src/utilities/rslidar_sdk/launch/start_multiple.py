import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch multiple rslidar_sdk nodes, one per lidar defined in config_multiple.yaml.
    Each lidar runs as a separate process with its own configuration.
    """
    package_share_dir = get_package_share_directory('rslidar_sdk')
    config_file = os.path.join(package_share_dir, 'config', 'config_multiple.yaml')

    # Load the multi-lidar configuration
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    nodes = []
    common_config = config.get('common', {})
    lidars = config.get('lidars', [])

    for lidar in lidars:
        lidar_name = lidar.get('name', 'lidar')
        driver_config = lidar.get('driver', {})
        ros_config = lidar.get('ros', {})

        # Build per-lidar config in the format expected by rslidar_sdk_node
        # The node expects a config file path, so we create individual temp configs
        single_lidar_config = {
            'common': common_config,
            'lidar': [{
                'driver': driver_config,
                'ros': ros_config
            }]
        }

        # Write temporary config file for this lidar
        temp_config_path = f'/tmp/rslidar_{lidar_name}_config.yaml'
        with open(temp_config_path, 'w') as f:
            yaml.dump(single_lidar_config, f)

        # Create a node for this lidar
        node = Node(
            namespace=lidar_name,
            package='rslidar_sdk',
            executable='rslidar_sdk_node',
            name=f'rslidar_sdk_{lidar_name}',
            output='screen',
            parameters=[{'config_path': temp_config_path}]
        )
        nodes.append(node)

    return LaunchDescription(nodes)

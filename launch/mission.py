import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # load crazyflies
    crazyflies_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'crazyflies.yaml')

    with open(crazyflies_yaml, 'r') as ymlfile:
        crazyflies = yaml.safe_load(ymlfile)
    
    # load swarm_manager parameters
    config_yaml = os.path.join(
        get_package_share_directory('crazyswarm_application'),
        'launch',
        'config.yaml')

    with open(config_yaml, 'r') as ymlfile:
        config = yaml.safe_load(ymlfile)

    return LaunchDescription([
        Node(
            package='crazyswarm_application',
            executable='mission_node',
            name='mission_node',
            output='screen',
            parameters=[crazyflies, config]
        ),
    ])

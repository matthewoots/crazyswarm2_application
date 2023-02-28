import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
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
    
    environment_name = config["environment_file"]

    # load obstacles configuration
    # load april tags configuration
    environment_yaml = os.path.join(
        get_package_share_directory('crazyswarm_application'),
        'launch',
        'environment',
        environment_name)
    
    with open(environment_yaml, 'r') as ymlfile:
        environment = yaml.safe_load(ymlfile)    

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false'),
        Node(
            package='crazyswarm_application',
            executable='crazyswarm_application_node',
            name='crazyswarm_application_node',
            output='screen',
            parameters=[crazyflies, config, environment]
        ),
        Node(
            package='crazyswarm_application',
            executable='april_detection_proxy_node',
            condition=LaunchConfigurationEquals('sim','true'),
            name='april_detection_proxy_node',
            output='screen',
            parameters=[crazyflies, config]
        ),
    ])

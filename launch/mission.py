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

    dictionary = dict()
    files = os.listdir(
        os.path.join(get_package_share_directory('crazyswarm_application'),
        'launch', 'mission'))
    for i in range(len(files)):
        dictionary.update({i: files[i]})
        print(str(i), '. ', files[i])

    inp = input('What is your mission file?\n')
    try:
        print('Mission chosen is', dictionary[int(inp)])
    except KeyError:
        print('Wrong input range!')
        exit()
    
    mission_yaml = os.path.join(
        get_package_share_directory('crazyswarm_application'),
        'launch', 'mission', dictionary[int(inp)])
    
    with open(mission_yaml, 'r') as ymlfile:
        mission = yaml.safe_load(ymlfile)

    return LaunchDescription([
        Node(
            package='crazyswarm_application',
            executable='mission_node',
            name='mission_node',
            output='screen',
            parameters=[crazyflies, config, mission]
        ),
    ])

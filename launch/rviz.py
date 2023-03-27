import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node


def generate_launch_description():

    # get mesh path
    april_tag_path = os.path.join(
        get_package_share_directory('crazyswarm_application'),
        'meshes',
        'master.dae')

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

    # config = [environment] + [{'mesh_path': april_tag_path}]
    config = [environment] + [{'mesh_path': april_tag_path}] + [config]

    ld = LaunchDescription()

    vis_node = Node(
            package='crazyswarm_application',
            executable='visualization_node',
            name='visualization_node',
            output='screen',
            parameters=config
            )
    
    rviz_node = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('crazyswarm_application'), 'launch', 'config.rviz')],
            parameters=[{
                "use_sim_time": True,
            }]
            )

    ld.add_action(vis_node)  
    ld.add_action(rviz_node)

    return ld
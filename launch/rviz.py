import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    april_tag_path = os.path.join(
        get_package_share_directory('crazyswarm_application'),
        'meshes',
        'master.dae')
    
    # load april tags configuration
    april_yaml = os.path.join(
        get_package_share_directory('crazyswarm_application'),
        'launch',
        'config.yaml')
    
    with open(april_yaml, 'r') as ymlfile:
        april = yaml.safe_load(ymlfile)

    config = [april] + [{'mesh_path': april_tag_path}]

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
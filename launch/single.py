import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # [1] load crazyflies
    crazyflies_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'crazyflies.yaml')
    # [2] load swarm_manager parameters + load april tags configuration
    config_yaml = os.path.join(
        get_package_share_directory('crazyswarm_application'),
        'launch',
        'config.yaml')
    # [3] load apriltag
    cfg_yaml = os.path.join(
        get_package_share_directory('apriltag_ros'),
        'cfg', 'tags_36h11.yaml')
    # [4] construct motion_capture_configuration
    motion_capture_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config', 'motion_capture.yaml')
    
    # [1]
    with open(crazyflies_yaml, 'r') as ymlfile:
        crazyflies = yaml.safe_load(ymlfile)
    # [2]
    with open(config_yaml, 'r') as ymlfile:
        config = yaml.safe_load(ymlfile)
    # [3]
    with open(cfg_yaml, 'r') as ymlfile:
        cfg = yaml.safe_load(ymlfile)
    # [4]
    with open(motion_capture_yaml, 'r') as ymlfile:
        motion_capture = yaml.safe_load(ymlfile)

    robots_list = crazyflies['robots']

    ld = LaunchDescription()

    motion_capture_params = motion_capture["/motion_capture_tracking"]["ros__parameters"]
    motion_capture_params["rigid_bodies"] = dict()
    for key, value in crazyflies["robots"].items():
        type = crazyflies["robot_types"][value["type"]]
        if value["enabled"] and type["motion_capture"]["enabled"]:
            motion_capture_params["rigid_bodies"][key] =  {
                    "initial_position": value["initial_position"],
                    "marker": type["motion_capture"]["marker"],
                    "dynamics": type["motion_capture"]["dynamics"],
                }

    for x in robots_list.keys():
        # load calibration
        calibration_yaml = os.path.join(
            get_package_share_directory('apriltag_ros'),
            'calibration',
            x + '.yaml')
        
        with open(calibration_yaml, 'r') as ymlfile:
            calibration = yaml.safe_load(ymlfile)
        
        crazy_params = [crazyflies]
        calib_params = [calibration] + [crazyflies]
        cfg_params = [cfg]
        app_params = [crazyflies] + [config]

        camera_node = Node(
                package="apriltag_ros",
                executable="cf_opencv_publisher.py",
                name='cf_streamer_' + x,
                output="screen",
                parameters=calib_params
            )
        
        tag_node = Node(
                package="apriltag_ros",
                executable="apriltag_node",
                name='apriltag_node' + x,
                output="screen",
                remappings=[
                    ('image_rect', x + '/image'),
                    ('camera_info', x + '/camera_info'),
                    ('detections', x + '/tag'),
                ],
                # remappings=[
                #     ('image_rect', '/image_raw'),
                #     ('camera_info', '/camera_info'),
                # ],
                parameters=cfg_params
            )
        
        server_node = Node(
            package='crazyflie',
            executable='crazyflie_server',
            name='crazyflie_server',
            output='screen',
            parameters=crazy_params
        )
        
        app_node = Node(
            package='crazyswarm_application',
            executable='crazyswarm_application_node',
            name='crazyswarm_application_node',
            output='screen',
            parameters=app_params
            )

        motion_node = Node(
            package='motion_capture_tracking',
            executable='motion_capture_tracking_node',
            name='motion_capture_tracking',
            output='screen',
            parameters=[motion_capture_params]
        )
        
        ld.add_action(server_node)
        ld.add_action(motion_node)
        ld.add_action(camera_node)
        ld.add_action(tag_node)
        ld.add_action(app_node)

    return ld

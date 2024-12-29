import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    param_config = os.path.join(
        get_package_share_directory('articubot_one'), 'config', 'depth_image_to_laser_scan.yaml')
    return LaunchDescription([
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            # remappings=[('depth', '/omni_bot/front/depth/image_raw'),
            #             ('depth_camera_info', '/omni_bot/front/depth/camera_info')],
            remappings=[('depth', '/articubot_one/front/depth/image_raw'),
                        ('depth_camera_info', '/articubot_one/front/depth/camera_info')],
            parameters=[param_config])
    ])
import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    package_name='articubot_one'

    rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name),'launch','rsp.launch.py'
            )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'my_bot'],
                    output='screen')
    
    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')
    
    world_file = os.path.join(
        get_package_share_directory("articubot_one"),
        # "worlds", "simple_wall_following.world"
        # "worlds", "maze_3_6x6.world"
        # "worlds", "self_made_maze.world"
        # "worlds", "cafe.world"
        "worlds", "furnished_office.world"
    )
    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'world': world_file, 'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )
    # Map server
    map_server_config_path = os.path.join(
        get_package_share_directory('articubot_one'),
        'maps',
        'lidar_map2.yaml'
    )

    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_server_config_path},
                    {'publish_latched': True}])

    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}])
    

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/home/carlios/slam_bot_ws2/src/depth-camera-slam/rviz/map_loader.rviz']
    )

    return LaunchDescription([
        # Launch the map server to load the map


        start_lifecycle_manager_cmd,
        map_server_cmd,
        rviz2

    ])

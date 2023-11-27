import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_path, get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('params_file',
            default_value=os.path.join(get_package_share_directory("lingao_nav"), 'config', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use for nav2'),
        DeclareLaunchArgument('map',
            default_value=os.path.join(get_package_share_directory("lingao_slam"), 'maps', 'warehouse.yaml'),
            description='Full path to the map yaml file to load'),
        DeclareLaunchArgument('collision_monitor',
            default_value=os.path.join(get_package_share_directory("lingao_nav"), 'config', 'collision_monitor.yaml'),
            description='Full path to the collision monitor yaml file to load'),



        # NAVIGATION
        IncludeLaunchDescription(str(get_package_share_path('deliverybot_navigation2') / 'launch/bringup_launch.py'),
                                 launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                                                   'params_file': LaunchConfiguration('params_file'),
                                                   'map': LaunchConfiguration('map')}.items()),

        # COLLISION MONITOR 
        Node(package='nav2_collision_monitor', executable='collision_monitor', output='screen', emulate_tty=True, 
                                parameters=[{'params_file': LaunchConfiguration('collision_monitor'),
                                            'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
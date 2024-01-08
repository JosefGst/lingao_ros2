import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_path, get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('open_rviz', default_value='false', description='Open rviz if true'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('params_file',
            default_value=os.path.join(get_package_share_directory("lingao_nav"), 'config', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use for nav2'),
        DeclareLaunchArgument('slam_params_file',
            default_value=os.path.join(get_package_share_directory("lingao_slam"), 'config', 'localizer_params_online_async.yaml'),
            description='Full path to the ROS2 parameters file to use for slam'),
        DeclareLaunchArgument('collision_monitor',
            default_value=os.path.join(get_package_share_directory("lingao_nav"), 'config', 'collision_monitor.yaml'),
            description='Full path to the collision monitor yaml file to load'),
        DeclareLaunchArgument(name='rvizconfig', default_value=os.path.join(get_package_share_directory("lingao_nav"), 'config', 'nav.rviz'),
                                     description='Absolute path to rviz config file'),


        # LOCALIZATION
        IncludeLaunchDescription(str(get_package_share_path('lingao_slam') / 'launch/slam_launch.py'),
                                 launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                                                   'slam_params_file': LaunchConfiguration('slam_params_file'),
                                                   }.items()),

        # NAVIGATION
        IncludeLaunchDescription(str(get_package_share_path('lingao_nav') / 'launch/navigation_launch.py'),
                                 launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                                                   'params_file': LaunchConfiguration('params_file'),
                                                   }.items()),

        # COLLISION MONITOR 
        # Node(package='nav2_collision_monitor', executable='collision_monitor', output='screen', emulate_tty=True, 
        #                         parameters=[{'params_file': LaunchConfiguration('collision_monitor'),
        #                                     'use_sim_time': LaunchConfiguration('use_sim_time')}]),

        # rviz
        Node(condition=IfCondition(LaunchConfiguration('open_rviz')),
             package='rviz2', executable='rviz2', name='rviz2', output='screen', arguments=['-d', LaunchConfiguration('rvizconfig')],)
    ])
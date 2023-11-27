import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_path, get_package_share_directory


def generate_launch_description():
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('slam_params_file',
            default_value=os.path.join(get_package_share_directory("lingao_slam"), 'config', 'mapper_params_online_async.yaml'),
            description='Full path to the ROS2 parameters file to use for the slam_toolbox node'),
            
        IncludeLaunchDescription(str(get_package_share_path('slam_toolbox')/ 'launch'/ 'online_async_launch.py'),
                                 launch_arguments={ #'use_sim_time': LaunchConfiguration('use_sim_time'),
                                                   'slam_params_file': LaunchConfiguration('slam_params_file')
                                                   }.items()),
    ])
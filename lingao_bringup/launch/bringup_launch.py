from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_path
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os


def generate_launch_description():
    pub_odom_tf = LaunchConfiguration('pub_odom_tf', default='false')
    pkg_share = FindPackageShare(package='lingao_description').find('lingao_description')
    default_rviz_config_path = os.path.join(pkg_share, 'config/urdf.rviz')
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("lingao_description"), "urdf", "MiniUGV_10A.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    return LaunchDescription([
        DeclareLaunchArgument('open_rviz', default_value='true', description='Whether to open rviz.'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path), description='Absolute path to rviz config file'),

        # lingao serila communication with MCU board
        IncludeLaunchDescription(str(get_package_share_path('lingao_base')/ 'launch'/ 'lingao_base_launch.py'),
                                 launch_arguments={'use_imu': 'true',
                                                    'pub_odom_tf': pub_odom_tf,
                                                    'freq': '10'}.items()),
        # Robot State Publisher
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="both",
            parameters=[robot_description,{'use_sim_time': LaunchConfiguration('use_sim_time')}]),
        # Joint State Publisher
        Node(package='joint_state_publisher', executable='joint_state_publisher', parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]),

        # Robot launch file
        IncludeLaunchDescription(str(get_package_share_path('lingao_bringup')/ 'launch'/ 'robot_launch.py'),
                                 ),
        # RPLIDAR A2M8
        IncludeLaunchDescription(str(get_package_share_path('rplidar_ros')/ 'launch'/ 'rplidar_a2m8_launch.py'),
                                launch_arguments={'serial_port': '/dev/rplidar',
                                                  'frame_id': 'scan'}.items()),
        # GPS
        IncludeLaunchDescription(str(get_package_share_path('lingao_bringup')/ 'launch'/ 'gps'/ 'nmea_serial_driver_launch.py'),
                                ),
                                                         
        # Rviz2
        Node(
        condition=IfCondition(LaunchConfiguration('open_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    ])
    
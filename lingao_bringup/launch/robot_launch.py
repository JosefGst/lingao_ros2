from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
from launch.substitutions import LaunchConfiguration


"""
Launches all the raw data processing & filtering
"""
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),

        # IMU COMPLEMENTARY FILTER
        IncludeLaunchDescription(str(get_package_share_path('lingao_bringup')/ 'launch'/'imu'/ 'complementary_filter_launch.py'),),

        # ROBOT LOCALIZATION
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
            parameters=[str(get_package_share_path('lingao_bringup') / 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
    ])
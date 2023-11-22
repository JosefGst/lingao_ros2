from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(str(get_package_share_path('lingao_base')/ 'launch'/ 'lingao_base_launch.py'),
                                 launch_arguments={'use_imu': 'true'}.items()),
    ])
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(str(get_package_share_path('lingao_bringup')/ 'launch'/'imu'/ 'complementary_filter_launch.py'),
                                 ),
    ])
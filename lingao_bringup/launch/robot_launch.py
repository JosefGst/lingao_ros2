from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path,get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os


"""
Launches all the raw data processing & filtering
"""
def generate_launch_description():
    gps_wpf_dir = get_package_share_directory(
        "lingao_bringup")
    rl_params_file = os.path.join(
        gps_wpf_dir, "config", "dual_ekf_navsat_params.yaml")
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),

        #! Complementary Filter not working well
        # IMU COMPLEMENTARY FILTER 
        # IncludeLaunchDescription(str(get_package_share_path('lingao_bringup')/ 'launch'/'imu'/ 'complementary_filter_launch.py'),),

        # IMU MADWICK FILTER
        IncludeLaunchDescription(str(get_package_share_path('lingao_bringup')/ 'launch'/'imu'/ 'madwick_filter_launch.py'),),

        # ROBOT LOCALIZATION
        # Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
        #     parameters=[str(get_package_share_path('lingao_bringup') / 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        # ),

        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_odom",
            output="screen",
            parameters=[rl_params_file, {"use_sim_time": True}],
            remappings=[("odometry/filtered", "odometry/local")],
            ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_map",
            output="screen",
            parameters=[rl_params_file, {"use_sim_time": True}],
            remappings=[("odometry/filtered", "odometry/global")],
            ),
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            output="screen",
            parameters=[rl_params_file, {"use_sim_time": True}],
            remappings=[
                ("imu/data", "imu/data"),
                ("gps/fix", "gps/fix"),
                ("gps/filtered", "gps/filtered"),
                ("odometry/gps", "odometry/gps"),
                ("odometry/filtered", "odometry/global"),
                ],
            ),
    ])
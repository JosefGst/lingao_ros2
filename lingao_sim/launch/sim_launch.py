# from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_path, get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg_share = launch_ros.substitutions.FindPackageShare(package='lingao_description').find('lingao_description')
    pkg_share = launch_ros.substitutions.FindPackageShare(package='lingao_description').find('lingao_description')
    default_model_path = os.path.join(pkg_share, 'urdf/MiniUGV_10A.xacro') 
    default_rviz_config_path = os.path.join(pkg_share, 'config/urdf.rviz')
    aws_small_warehouse_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')

    gui_arg = DeclareLaunchArgument(name='gui', default_value='false', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)
    headless = DeclareLaunchArgument(name='headless', default_value='True', choices=['True', 'False'],
                                    description='Flag to enable Gazebo headless mode')
    gazebo_params_path = DeclareLaunchArgument('gazebo_params_file',
            default_value=os.path.join(aws_small_warehouse_dir, 'config', 'gazebo_params.yaml'),
            description='Full path to the gazebo parameters file increase pub rate')

    aws_warehouse = IncludeLaunchDescription(str(get_package_share_path("aws_robomaker_small_warehouse_world")/ "launch"/ "small_warehouse.launch.py"),
                                             launch_arguments={'headless': LaunchConfiguration('headless'),
                                                               'params_file': str(get_package_share_path('aws_robomaker_small_warehouse_world')/ 'config'/ 'gazebo_params.yaml')}.items())

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
                    'rate': 50}],
        # condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    # joint_state_publisher_gui_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     condition=IfCondition(LaunchConfiguration('gui'))
    # )

    spawn_entity = launch_ros.actions.Node(
    	package='gazebo_ros', 
    	executable='spawn_entity.py',
        arguments=['-entity', 'sam_bot', '-topic', 'robot_description'],
        output='screen'
    )

    # Robot launch file
    robot_launch = IncludeLaunchDescription(str(get_package_share_path('lingao_bringup')/ 'launch'/ 'robot_launch.py'),
                                            launch_arguments={'use_sim_time': use_sim_time}.items())
    
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('open_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        headless,
        gazebo_params_path,
        aws_warehouse,
        DeclareLaunchArgument('open_rviz', default_value='False'),

        # ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen'),
        gui_arg,
        model_arg,
        joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        robot_state_publisher_node,
        spawn_entity,
        robot_launch,
        rviz_arg,
        rviz_node
    ])
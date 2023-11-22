from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port_name', default_value='/dev/lingao', description='The name of the port for communication.'),
        DeclareLaunchArgument('port_baud', default_value='230400', description='The baud rate of the port.'),
        DeclareLaunchArgument('cmd_vel_sub_timeout', default_value='1000', description='The timeout of cmd_vel subscriber in ms.'),
        DeclareLaunchArgument('pub_odom_tf', default_value='false', description='Whether to publish odom tf.'),
        DeclareLaunchArgument('use_imu', default_value='true', description='Whether to use imu.'),

        Node(name='lingao_base_node',
            package='lingao_base',
            executable='lingao_base_node',
            parameters=[{'port_name': LaunchConfiguration('port_name')},
                        {'port_baud': LaunchConfiguration('port_baud')},
                        {'name': 'freq',               'description': 'The frequency of odom and imu in HZ.',          'default_value': 80},
                        {'name': 'topic_cmd_vel_name', 'description': 'The name of the topic for cmd_vel.',            'default_value': 'cmd_vel'},
                        {'name': 'publish_odom_name',  'description': 'The name of the topic for publishing odom.',    'default_value': 'odom'},
                        {'name': 'base_frame_id',      'description': 'The ID of the base frame.',                     'default_value': 'base_footprint'},
                        {'cmd_vel_sub_timeout' : LaunchConfiguration('cmd_vel_sub_timeout')},
                        {'pub_odom_tf': LaunchConfiguration('pub_odom_tf')},
                        {'name': 'linear_scale',       'description': 'The scale of linear velocity.',                 'default_value': 1.0},
                        {'name': 'angular_scale',      'description': 'The scale of angular velocity.',                'default_value': 1.0},
                        {'name': 'topic_imu',          'description': 'The name of the topic for imu.',                'default_value': 'imu_raw'},
                        {'name': 'imu_frame_id',       'description': 'The ID of the imu frame.',                      'default_value': 'imu_link'},
                        {'use_imu': LaunchConfiguration('use_imu')},
                        {'name': 'imu_calibrate_gyro', 'description': 'Whether to calibrate gyro.',                    'default_value': True},
                        {'name': 'imu_calib_samples',  'description': 'The number of samples for gyro calibration.',   'default_value': 300},
            ]),
    ])
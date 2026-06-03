from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('techx_r2_chassis_control')
    default_config = os.path.join(pkg_share, 'config', 'mock_chassis_serial.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('host_port', default_value='/tmp/r2_chassis_host'),
        DeclareLaunchArgument('stm32_port', default_value='/tmp/r2_chassis_stm32'),
        DeclareLaunchArgument('simulate_error', default_value='false'),
        DeclareLaunchArgument('drop_first_ack', default_value='false'),
        Node(
            package='techx_r2_chassis_control',
            executable='mock_chassis_stm32',
            name='mock_chassis_stm32',
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {
                    'port': LaunchConfiguration('stm32_port'),
                    'simulate_error': LaunchConfiguration('simulate_error'),
                    'drop_first_ack': LaunchConfiguration('drop_first_ack'),
                },
            ],
        ),
        Node(
            package='techx_r2_chassis_control',
            executable='chassis_serial_node',
            name='chassis_serial_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {'port': LaunchConfiguration('host_port')},
            ],
        ),
    ])

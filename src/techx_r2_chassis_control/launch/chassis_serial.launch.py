from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('techx_r2_chassis_control')
    default_config = os.path.join(pkg_share, 'config', 'chassis_serial.yaml')

    config_arg = DeclareLaunchArgument('config', default_value=default_config)
    port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyUSB0')

    return LaunchDescription([
        config_arg,
        port_arg,
        Node(
            package='techx_r2_chassis_control',
            executable='chassis_serial_node',
            name='chassis_serial_node',
            output='screen',
            parameters=[LaunchConfiguration('config'), {'port': LaunchConfiguration('port')}],
        ),
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host_port = LaunchConfiguration('host_port')
    stm32_port = LaunchConfiguration('stm32_port')
    baudrate = LaunchConfiguration('baudrate')
    simulate_error = LaunchConfiguration('simulate_error')
    drop_first_ack = LaunchConfiguration('drop_first_ack')

    socat_process = ExecuteProcess(
        cmd=[
            'socat',
            '-d',
            '-d',
            'pty,raw,echo=0,link=/tmp/r2_chassis_host',
            'pty,raw,echo=0,link=/tmp/r2_chassis_stm32',
        ],
        output='screen'
    )

    chassis_serial_node = Node(
        package='techx_r2_chassis_control',
        executable='chassis_serial_node',
        name='chassis_serial_node',
        output='screen',
        parameters=[{
            'port': host_port,
            'baudrate': baudrate,
            'velocity_rate_hz': 30.0,
            'cmd_vel_timeout_sec': 0.2,
            'task_ack_timeout_sec': 0.1,
            'task_max_retries': 3,
            'send_zero_in_task_mode': True,
            'estop_repeat_count': 3,
        }]
    )

    mock_chassis_stm32 = Node(
        package='techx_r2_chassis_control',
        executable='mock_chassis_stm32',
        name='mock_chassis_stm32',
        output='screen',
        parameters=[{
            'port': stm32_port,
            'baudrate': baudrate,
            'simulate_error': simulate_error,
            'drop_first_ack': drop_first_ack,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'host_port',
            default_value='/tmp/r2_chassis_host',
            description='Virtual serial port used by chassis_serial_node'
        ),
        DeclareLaunchArgument(
            'stm32_port',
            default_value='/tmp/r2_chassis_stm32',
            description='Virtual serial port used by mock_chassis_stm32'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='115200',
            description='Serial baudrate'
        ),
        DeclareLaunchArgument(
            'simulate_error',
            default_value='false',
            description='Mock returns ERROR instead of DONE'
        ),
        DeclareLaunchArgument(
            'drop_first_ack',
            default_value='false',
            description='Mock drops first ACK to test resend'
        ),

        socat_process,

        TimerAction(
            period=0.5,
            actions=[
                mock_chassis_stm32,
                chassis_serial_node,
            ]
        ),
    ])
from setuptools import setup
from glob import glob
import os

package_name = 'techx_r2_chassis_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='techx',
    maintainer_email='todo@example.com',
    description='Python serial bridge for R2 chassis STM32.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chassis_serial_node = techx_r2_chassis_control.chassis_serial_node:main',
            'mock_chassis_stm32 = techx_r2_chassis_control.mock_chassis_stm32:main',
            'test_step_command_client = techx_r2_chassis_control.test_step_command_client:main',
            'test_cmd_vel_pub = techx_r2_chassis_control.test_cmd_vel_pub:main',
            'protocol_demo = techx_r2_chassis_control.protocol_demo:main',
            'test_estop_client = techx_r2_chassis_control.test_estop_client:main',
            'test_lift_control_client = techx_r2_chassis_control.test_lift_control_client:main',
        ],
    },
)

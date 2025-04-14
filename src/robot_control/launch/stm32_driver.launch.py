from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_descriptions():
    return LaunchDescription([
        Node(
            package='robot_control',
            executable='stm32_driver',
            name='stm32_driver',
            parameters=[{
                'serial_port': '/dev/ttyAMA10',
                'baud_rate': 115200
            }],
            output='screen'
        )
    ])
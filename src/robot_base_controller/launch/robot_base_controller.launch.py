from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_base_controller',
            executable='inverse_kinetic',
            name='inverse_kinetic',
            output='screen'
        ),
        Node(
            package='robot_base_controller',
            executable='connect_node',
            name='node_serial',
            output='screen'
        ),
        Node(
            package='robot_base_controller',
            executable='gpio_encoder_node',
            name='read_encoder_odom',
            output='screen'
        )
    ])

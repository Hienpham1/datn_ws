from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trajectory_planner',
            executable='duong_thang',
            name='duong_thang',
            output='screen'
        ),
        Node(
            package='trajectory_planner',
            executable='pose_array_visualizer',
            name='visualizer_node',
            output='screen'
        )
    ])

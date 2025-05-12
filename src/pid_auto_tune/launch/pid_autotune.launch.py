from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    bag_path = os.path.expanduser('~/ros2_ws/bags/your_bag')  # chỉnh lại đường dẫn rosbag của bạn

    return LaunchDescription([
        # Chạy rosbag play --loop
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_path, '--loop'],
            output='screen'
        ),

        # Chạy node data_collector
        Node(
            package='pid_autotuner',
            executable='data_collector.py',
            name='pid_data_collector',
            output='screen'
        )
    ])

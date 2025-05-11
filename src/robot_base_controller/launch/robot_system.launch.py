from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('robot_base_controller'),
            'config',
            'ekf.yaml'
        ])],
        remappings=[
            ('/odometry/filtered', '/odom_fused'),
            ('/odometry/encoder', '/odom_encoder')
        ]
    )

    inverse_kinetic = Node(
        package='robot_base_controller',
        executable='inverse_kinetic_node',
        name='inverse_kinetic_node',
        output='screen'
    )

    serial_node = Node(
        package='robot_base_controller',
        executable='node_serial',
        name='node_serial',
        output='screen'
    )

    encoder_node = Node(
        package='robot_base_controller',
        executable='read_encoder_node',
        name='read_encoder_node',
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('robot_base_controller'),
            'rviz',
            'robot_system.rviz'  # nếu có sẵn config rviz
        ])],
        output='screen',
        condition=None  # bỏ nếu bạn muốn bật RViz luôn
    )

    return LaunchDescription([
        inverse_kinetic,
        serial_node,
        encoder_node,
        ekf_node,
        rviz
    ])

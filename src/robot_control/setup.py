from setuptools import find_packages, setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='datn',
    maintainer_email='datn@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'connect_node = robot_control.node_serial:main',
            'inverse_kinetic = robot_control.inverse_kinetic:main',
            'square_controller = robot_control.square_controller:main',
            'pose_to_velocity_node = robot_control.hinh_vuong:main',
            'pid_input_node = robot_control.pid_input_node:main',
            'connect_node_pwm = robot_control.connect_node1:main',
            'inverse_kinetic_pwm = robot_control.inverse_kinetic1:main',
            'pd_pose_controller = robot_control.pd_pose_controller:main',
            'path_tracker = robot_control.path_tracker_node:main'
        ],
    },
)

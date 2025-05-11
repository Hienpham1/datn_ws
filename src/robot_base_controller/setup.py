from setuptools import find_packages, setup

package_name = 'robot_base_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot_base_controller.launch.py']),
        ('share/' + package_name + '/config', ['config/ekf.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='datn',
    maintainer_email='vanhienpham96@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inverse_kinetic = robot_base_controller.inverse_kinetic:main',
            'gpio_encoder_node = robot_base_controller.read_encoder_odom:main',
            'connect_node = robot_base_controller.node_serial:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'robotvevach'

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
    maintainer_email='vanhienpham96@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inverse_kinetic = robotvevach.inverse_kinetic:main',
            'gpio_encoder_node = robotvevach.read_encoder_odom:main',
            'connect_node = robotvevach.node_serial:main',
            'pd_pose_controller = robotvevach.pd_pose_controller:main',
        ],
    },
)

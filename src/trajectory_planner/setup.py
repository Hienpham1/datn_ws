from setuptools import find_packages, setup

package_name = 'trajectory_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/trajectory_planner.launch.py']),
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
            'planner_node = trajectory_planner.planner_node:main',
            'pose_array_visualizer = trajectory_planner.visualizer_node:main',
            'duong_thang = trajectory_planner.duong_thang:main'
        ],
    },
)

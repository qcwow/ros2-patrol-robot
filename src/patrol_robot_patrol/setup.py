from glob import glob
from setuptools import find_packages, setup


package_name = 'patrol_robot_patrol'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/behavior_trees', glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Patrol Robot Maintainer',
    maintainer_email='maintainer@example.com',
    description='Multi-waypoint patrol manager for Nav2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'patrol_manager = patrol_robot_patrol.patrol_manager:main',
            'frontier_explorer = patrol_robot_patrol.frontier_explorer:main',
            'navigation_health_monitor = patrol_robot_patrol.navigation_health_monitor:main',
            'navigation_regression_recorder = patrol_robot_patrol.navigation_regression_recorder:main',
            'navigation_acceptance_runner = patrol_robot_patrol.navigation_acceptance_runner:main',
            'map_artifact_validator = patrol_robot_patrol.map_artifact_validator:main',
            'rotation_diagnostic_recorder = patrol_robot_patrol.rotation_diagnostic_recorder:main',
            'manual_lidar_safety = patrol_robot_patrol.manual_lidar_safety:main',
            'base_command_watchdog = patrol_robot_patrol.base_command_watchdog:main',
            'safe_joystick_teleop = patrol_robot_patrol.safe_joystick_teleop:main',
        ],
    },
)

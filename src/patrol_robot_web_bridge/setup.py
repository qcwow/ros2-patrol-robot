from setuptools import find_packages, setup

package_name = 'patrol_robot_web_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Patrol Robot Maintainer',
    maintainer_email='maintainer@example.com',
    description='Safe HTTP gateway between the patrol web console and ROS 2.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['web_bridge = patrol_robot_web_bridge.bridge_node:main']},
)

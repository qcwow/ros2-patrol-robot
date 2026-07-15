from setuptools import find_packages, setup


package_name = 'patrol_robot_simulator'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/lightweight_simulation.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Patrol Robot Maintainer',
    maintainer_email='maintainer@example.com',
    description='Lightweight 2D simulator for Nav2 patrol development.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'simulator = patrol_robot_simulator.simulator_node:main',
        ],
    },
)


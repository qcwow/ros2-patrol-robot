from glob import glob
from setuptools import find_packages, setup


package_name = 'patrol_robot_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Patrol Robot Maintainer',
    maintainer_email='maintainer@example.com',
    description='Registered RGB-D processing and filtered point cloud generation.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'rgbd_processor = patrol_robot_camera.rgbd_processor_node:main',
            'local_voxel_mapper = '
            'patrol_robot_camera.local_voxel_mapper_node:main',
        ],
    },
)

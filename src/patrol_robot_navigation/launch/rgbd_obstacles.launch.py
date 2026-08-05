"""Segment RGB-D ground points before feeding the Nav2 local costmap."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    navigation_share = FindPackageShare('patrol_robot_navigation')
    default_params = PathJoinSubstitution([
        navigation_share,
        'config',
        'rgbd_obstacles_real_car.yaml',
    ])

    detector = Node(
        package='rtabmap_util',
        executable='obstacles_detection',
        name='obstacles_detection',
        output='screen',
        parameters=[LaunchConfiguration('obstacle_params_file')],
        remappings=[
            ('cloud', LaunchConfiguration('input_cloud')),
            ('obstacles', LaunchConfiguration('obstacles_cloud')),
            ('ground', LaunchConfiguration('ground_cloud')),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'obstacle_params_file', default_value=default_params),
        DeclareLaunchArgument(
            'input_cloud', default_value='/camera/points/filtered'),
        DeclareLaunchArgument(
            'obstacles_cloud', default_value='/camera/obstacles'),
        DeclareLaunchArgument(
            'ground_cloud', default_value='/camera/ground'),
        detector,
    ])

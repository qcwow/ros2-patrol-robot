from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_topic = LaunchConfiguration('map_topic')
    navigation_share = FindPackageShare('patrol_robot_navigation')

    default_params = PathJoinSubstitution(
        [navigation_share, 'config', 'slam_toolbox.yaml']
    )

    slam = GroupAction([
        # Keep the SLAM and static-map publishers private. map_source_mux is
        # the sole owner of the canonical /map topic consumed by Nav2 and RViz.
        SetRemap(src='/map', dst=map_topic),
        SetRemap(src='map', dst=map_topic),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch',
                    'online_async_launch.py',
                ])
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': params_file,
            }.items(),
        ),
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('map_topic', default_value='/slam_map'),
        slam,
    ])

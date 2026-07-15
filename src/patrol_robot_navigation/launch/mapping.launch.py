from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    navigation_share = FindPackageShare('patrol_robot_navigation')

    default_params = PathJoinSubstitution(
        [navigation_share, 'config', 'slam_toolbox.yaml']
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': params_file,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        slam,
    ])

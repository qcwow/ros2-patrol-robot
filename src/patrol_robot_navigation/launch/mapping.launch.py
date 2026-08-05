from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_topic = LaunchConfiguration('map_topic')
    navigation_share = FindPackageShare('patrol_robot_navigation')

    default_params = PathJoinSubstitution(
        [navigation_share, 'config', 'slam_toolbox.yaml']
    )

    # Humble's async SLAM node is not a lifecycle node and has no map reset
    # service. The supervisor owns its process so static AMCL mode can stop its
    # map->odom publisher completely and every reset starts a fresh pose graph.
    slam = Node(
        package='patrol_robot_web_bridge',
        executable='slam_session_manager',
        name='slam_session_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'slam_params_file': params_file,
            'map_topic': map_topic,
            'startup_active': True,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('map_topic', default_value='/slam_map'),
        slam,
    ])

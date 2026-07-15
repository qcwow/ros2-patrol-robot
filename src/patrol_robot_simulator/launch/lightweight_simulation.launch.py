from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')

    description_share = FindPackageShare('patrol_robot_description')
    navigation_share = FindPackageShare('patrol_robot_navigation')
    robot_xacro = PathJoinSubstitution(
        [description_share, 'urdf', 'patrol_robot.urdf.xacro']
    )
    default_map = PathJoinSubstitution(
        [navigation_share, 'maps', 'pipeline_map.yaml']
    )
    robot_description = ParameterValue(
        Command(['xacro ', robot_xacro]),
        value_type=str,
    )

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    simulator = Node(
        package='patrol_robot_simulator',
        executable='simulator',
        name='lightweight_simulator',
        output='screen',
        parameters=[{
            'map_yaml': map_yaml,
            'initial_x': -6.0,
            'initial_y': -4.0,
            'initial_yaw': 0.0,
            'scan_samples': 240,
            'scan_rate': 10.0,
            'linear_speed_limit': 1.5,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map),
        state_publisher,
        simulator,
    ])

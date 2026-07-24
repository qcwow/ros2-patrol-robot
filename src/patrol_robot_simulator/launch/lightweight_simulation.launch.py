from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_yaw = LaunchConfiguration('initial_yaw')
    scan_samples = LaunchConfiguration('scan_samples')
    scan_rate = LaunchConfiguration('scan_rate')

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
            'initial_x': ParameterValue(initial_x, value_type=float),
            'initial_y': ParameterValue(initial_y, value_type=float),
            'initial_yaw': ParameterValue(initial_yaw, value_type=float),
            'scan_samples': ParameterValue(scan_samples, value_type=int),
            'scan_rate': ParameterValue(scan_rate, value_type=float),
            'linear_speed_limit': 1.5,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('initial_x', default_value='-6.0'),
        DeclareLaunchArgument('initial_y', default_value='-4.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('scan_samples', default_value='240'),
        DeclareLaunchArgument('scan_rate', default_value='10.0'),
        state_publisher,
        simulator,
    ])

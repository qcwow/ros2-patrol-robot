from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')

    description_share = FindPackageShare('patrol_robot_description')
    gazebo_share = FindPackageShare('patrol_robot_gazebo')
    robot_xacro = PathJoinSubstitution(
        [description_share, 'urdf', 'patrol_robot.urdf.xacro']
    )
    world = PathJoinSubstitution([gazebo_share, 'worlds', 'pipeline_world.sdf'])
    bridge_config = PathJoinSubstitution([gazebo_share, 'config', 'bridge.yaml'])

    robot_description = ParameterValue(
        Command(['xacro ', robot_xacro]),
        value_type=str,
    )

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
            )
        ),
        condition=UnlessCondition(headless),
        launch_arguments={
            'gz_args': ['-r -v 2 ', world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
            )
        ),
        condition=IfCondition(headless),
        launch_arguments={
            'gz_args': ['-r -s --headless-rendering -v 2 ', world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description, 'use_sim_time': use_sim_time}
        ],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'patrol_robot',
            '-topic', 'robot_description',
            '-x', '-6.0',
            '-y', '-4.0',
            '-z', '0.02',
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config}],
    )

    scene_sync = Node(
        package='patrol_robot_gazebo',
        executable='gazebo_scene_sync',
        name='gazebo_scene_sync',
        output='screen',
        parameters=[{
            'world_name': 'pipeline_inspection',
            'scenario_topic': '/patrol/map_scenario',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        gazebo_gui,
        gazebo_headless,
        state_publisher,
        spawn_robot,
        bridge,
        scene_sync,
    ])

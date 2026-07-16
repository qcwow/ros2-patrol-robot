from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    patrol_autostart = LaunchConfiguration('patrol_autostart')
    loop = LaunchConfiguration('loop')
    map_yaml = LaunchConfiguration('map')
    headless = LaunchConfiguration('headless')
    start_rviz = LaunchConfiguration('start_rviz')
    use_gazebo = LaunchConfiguration('use_gazebo')
    waypoints = LaunchConfiguration('waypoints')

    gazebo_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('patrol_robot_gazebo'), 'launch', 'simulation.launch.py']
            )
        ),
        condition=IfCondition(use_gazebo),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'headless': headless,
        }.items(),
    )

    camera_processing = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('patrol_robot_camera'),
                'launch',
                'camera_processing.launch.py',
            ])
        ),
        condition=IfCondition(use_gazebo),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    lightweight_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('patrol_robot_simulator'),
                'launch',
                'lightweight_simulation.launch.py',
            ])
        ),
        condition=UnlessCondition(use_gazebo),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('patrol_robot_navigation'), 'launch', 'navigation.launch.py']
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml,
        }.items(),
    )

    patrol_manager = Node(
        package='patrol_robot_patrol',
        executable='patrol_manager',
        name='patrol_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'waypoint_file': waypoints,
            'autostart': patrol_autostart,
            'loop': loop,
            'start_delay_seconds': 10.0,
            'goal_timeout_seconds': 120.0,
            'max_retries': 1,
            'retry_delay_seconds': 3.0,
            'stop_on_failure': False,
        }],
    )

    web_bridge = Node(
        package='patrol_robot_web_bridge',
        executable='web_bridge',
        name='patrol_robot_web_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'http_host': '0.0.0.0',
            'http_port': 8765,
            'max_linear_speed': 0.6,
            'max_angular_speed': 0.8,
            'manual_command_timeout': 0.5,
            'perception_initial_mode': 'fusion',
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(start_rviz),
        arguments=[
            '-d',
            PathJoinSubstitution(
                [FindPackageShare('nav2_bringup'), 'rviz', 'nav2_default_view.rviz']
            ),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    default_waypoints = PathJoinSubstitution(
        [FindPackageShare('patrol_robot_patrol'), 'config', 'waypoints.yaml']
    )
    default_map = PathJoinSubstitution(
        [FindPackageShare('patrol_robot_navigation'), 'maps', 'pipeline_map.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('patrol_autostart', default_value='true'),
        DeclareLaunchArgument('loop', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        # The full simulator is the default because cameras and editable 3D
        # collision entities only exist in Gazebo.  Resource-constrained runs
        # can still opt into the lightweight simulator with use_gazebo:=false.
        DeclareLaunchArgument('use_gazebo', default_value='true'),
        DeclareLaunchArgument('waypoints', default_value=default_waypoints),
        gazebo_simulation,
        camera_processing,
        lightweight_simulation,
        navigation,
        patrol_manager,
        web_bridge,
        rviz,
    ])

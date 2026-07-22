from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
    localization_mode = LaunchConfiguration('localization_mode')
    record_navigation_metrics = LaunchConfiguration('record_navigation_metrics')
    regression_scenario = LaunchConfiguration('regression_scenario')
    ground_truth_localization = PythonExpression([
        "'", use_gazebo, "' != 'true' or '",
        localization_mode, "' == 'ground_truth'",
    ])
    use_ekf_localization = PythonExpression([
        "'", use_gazebo, "' == 'true' and '",
        localization_mode, "' == 'ekf'",
    ])
    odom_pose_is_world = PythonExpression([
        "'", use_gazebo, "' == 'true' and '",
        localization_mode, "' == 'ground_truth'",
    ])

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
            'ground_truth_odometry': ground_truth_localization,
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
            'ground_truth_localization': 'true',
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
            'ground_truth_localization': ground_truth_localization,
        }.items(),
    )

    ekf_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        condition=IfCondition(use_ekf_localization),
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('patrol_robot_bringup'),
                'config',
                'ekf_sim.yaml',
            ]),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[('odometry/filtered', '/odom')],
    )

    navigation_health = Node(
        package='patrol_robot_patrol',
        executable='navigation_health_monitor',
        name='navigation_health',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # Ground-truth simulation still has to prove that sensors, odom,
            # TF and every Nav2 lifecycle node are live. AMCL covariance is
            # enabled by the real-robot profile after map localization owns TF.
            'require_amcl_covariance': ParameterValue(
                use_ekf_localization,
                value_type=bool,
            ),
            # The configured global map rate is 0.5 Hz, but Gazebo software
            # rendering can stretch it to roughly 0.33 Hz. Eight seconds still
            # detects a stopped publisher while avoiding a 3-second boundary
            # race on a busy VM.
            'costmap_timeout_seconds': 8.0,
            'footprint_safety_margin': 0.01,
        }],
    )

    regression_recorder = Node(
        package='patrol_robot_patrol',
        executable='navigation_regression_recorder',
        name='navigation_regression_recorder',
        output='screen',
        condition=IfCondition(record_navigation_metrics),
        parameters=[{
            'use_sim_time': use_sim_time,
            'scenario': regression_scenario,
        }],
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
            'loop_count': 1,
            'start_delay_seconds': 10.0,
            'goal_timeout_seconds': 120.0,
            'max_retries': 2,
            'retry_delay_seconds': 2.0,
            # Health recoveries are counted consecutively. After a healthy
            # low-speed stretch the patrol manager clears the old streak.
            'max_health_recoveries': 3,
            'health_recovery_reset_stable_seconds': 5.0,
            'health_recovery_reset_progress_meters': 0.5,
            'lap_restart_delay_seconds': 1.5,
            'action_name': 'navigate_to_pose',
            'behavior_tree': PathJoinSubstitution([
                FindPackageShare('patrol_robot_patrol'),
                'behavior_trees',
                'navigate_to_pose_stable.xml',
            ]),
            'behavior_tree_no_spin': PathJoinSubstitution([
                FindPackageShare('patrol_robot_patrol'),
                'behavior_trees',
                'navigate_to_pose_no_spin.xml',
            ]),
            'behavior_tree_restricted': PathJoinSubstitution([
                FindPackageShare('patrol_robot_patrol'),
                'behavior_trees',
                'navigate_to_pose_restricted.xml',
            ]),
            'require_navigation_health': True,
            'enforce_required_sensors': ParameterValue(
                use_gazebo,
                value_type=bool,
            ),
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
            # Follow the Nav2 / TurtleBot3 baseline: lidar owns base collision
            # avoidance. RGB-D stays available for inspection and may be
            # explicitly enabled as a navigation source from the web console.
            'perception_initial_mode': 'lidar',
            # Gazebo spawns the robot here; its /odom starts at zero. The web
            # bridge uses this offset to reinitialize AMCL after a map switch.
            'simulation_origin_x': -6.0,
            'simulation_origin_y': -4.0,
            'simulation_origin_yaw': 0.0,
            'odom_pose_is_world': ParameterValue(
                odom_pose_is_world,
                value_type=bool,
            ),
            'ground_truth_localization': ParameterValue(
                ground_truth_localization,
                value_type=bool,
            ),
            # Gazebo starts at a known pose. Seed AMCL after EKF odometry is
            # live so the health gate can validate covariance before motion.
            # Real-robot bringup must leave this disabled and obtain an
            # operator- or fiducial-confirmed initial pose instead.
            'seed_initial_pose_at_start': ParameterValue(
                use_ekf_localization,
                value_type=bool,
            ),
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
        # The web UI owns task creation and start. Waiting here prevents the
        # packaged fallback route from running before a selected map is applied.
        DeclareLaunchArgument('patrol_autostart', default_value='false'),
        DeclareLaunchArgument('loop', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        # The full simulator is the default because cameras and editable 3D
        # collision entities only exist in Gazebo.  Resource-constrained runs
        # can still opt into the lightweight simulator with use_gazebo:=false.
        DeclareLaunchArgument('use_gazebo', default_value='true'),
        DeclareLaunchArgument(
            'localization_mode',
            default_value='ekf',
            description='ekf uses wheel odom + IMU + AMCL; ground_truth is diagnostic only',
        ),
        DeclareLaunchArgument('record_navigation_metrics', default_value='false'),
        DeclareLaunchArgument('regression_scenario', default_value='manual_run'),
        DeclareLaunchArgument('waypoints', default_value=default_waypoints),
        gazebo_simulation,
        camera_processing,
        lightweight_simulation,
        ekf_localization,
        navigation,
        navigation_health,
        regression_recorder,
        patrol_manager,
        web_bridge,
        rviz,
    ])

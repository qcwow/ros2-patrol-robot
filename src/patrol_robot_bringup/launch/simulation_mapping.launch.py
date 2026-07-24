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
    headless = LaunchConfiguration('headless')
    start_rviz = LaunchConfiguration('start_rviz')
    use_gazebo = LaunchConfiguration('use_gazebo')
    enable_3d_mapping = LaunchConfiguration('enable_3d_mapping')
    enable_autonomous_exploration = LaunchConfiguration(
        'enable_autonomous_exploration'
    )
    autonomous_exploration_autostart = LaunchConfiguration(
        'autonomous_exploration_autostart'
    )
    patrol_autostart = LaunchConfiguration('patrol_autostart')
    patrol_loop = LaunchConfiguration('patrol_loop')
    waypoints = LaunchConfiguration('waypoints')
    ground_truth_odometry = LaunchConfiguration('ground_truth_odometry')
    map_yaml = LaunchConfiguration('map')

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
            'ground_truth_odometry': ground_truth_odometry,
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
            # Use the same denser scan stream selected during map-quality
            # tuning, but keep it inside the complete integrated stack.
            'scan_samples': '360',
            'scan_rate': '15.0',
        }.items(),
    )

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('patrol_robot_navigation'), 'launch', 'mapping.launch.py']
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map_topic': '/slam_map',
        }.items(),
    )

    map_source_mux = Node(
        package='patrol_robot_web_bridge',
        executable='map_source_mux',
        name='map_source_mux',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Keep the original static-map navigation capability alive inside the
    # unified SLAM launch. Its map is isolated on /static_map until the bridge
    # safely pauses SLAM localization and selects it through map_source_mux.
    static_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('patrol_robot_navigation'),
                'config',
                'nav2_params.yaml',
            ]),
            {
                'yaml_filename': map_yaml,
                'use_sim_time': use_sim_time,
            },
        ],
        remappings=[
            ('map', '/static_map'),
            ('map_metadata', '/static_map_metadata'),
        ],
    )

    static_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('patrol_robot_navigation'),
                'config',
                'nav2_params.yaml',
            ]),
            {
                'use_sim_time': use_sim_time,
                # SLAM Toolbox owns map -> odom initially. The bridge enables
                # AMCL TF only after a static map has loaded successfully.
                'tf_broadcast': False,
            },
        ],
        remappings=[
            ('map', '/static_map'),
            ('map_metadata', '/static_map_metadata'),
        ],
    )

    static_localization_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_static_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    octomap_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('patrol_robot_navigation'),
                'launch',
                'octomap_mapping.launch.py',
            ])
        ),
        condition=IfCondition(PythonExpression([
            "'", use_gazebo, "' == 'true' and '",
            enable_3d_mapping, "' == 'true'",
        ])),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    exploration_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('patrol_robot_navigation'),
                'launch',
                'exploration_navigation.launch.py',
            ])
        ),
        condition=IfCondition(enable_autonomous_exploration),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    frontier_explorer = Node(
        package='patrol_robot_patrol',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        condition=IfCondition(enable_autonomous_exploration),
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autonomous_exploration_autostart,
            'start_delay_seconds': 15.0,
            'goal_timeout_seconds': 120.0,
            'planning_period_seconds': 2.5,
            'max_unreachable_cycles': 8,
            'behavior_tree': PathJoinSubstitution([
                FindPackageShare('patrol_robot_patrol'),
                'behavior_trees',
                'navigate_to_frontier.xml',
            ]),
        }],
    )

    navigation_health = Node(
        package='patrol_robot_patrol',
        executable='navigation_health_monitor',
        name='navigation_health',
        output='screen',
        condition=IfCondition(enable_autonomous_exploration),
        parameters=[{
            'use_sim_time': use_sim_time,
            # Live SLAM owns map -> odom, so Mapping Mode does not need AMCL.
            'require_amcl_covariance': False,
            'costmap_timeout_seconds': 8.0,
            'footprint_safety_margin': 0.01,
            # The exploration stack intentionally omits map_server, AMCL and
            # waypoint_follower. PatrolManager only needs these six servers.
            'lifecycle_nodes': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'velocity_smoother',
                'bt_navigator',
            ],
        }],
    )

    patrol_manager = Node(
        package='patrol_robot_patrol',
        executable='patrol_manager',
        name='patrol_manager',
        output='screen',
        condition=IfCondition(enable_autonomous_exploration),
        parameters=[{
            'use_sim_time': use_sim_time,
            'waypoint_file': waypoints,
            'autostart': patrol_autostart,
            'loop': patrol_loop,
            'loop_count': 1,
            'start_delay_seconds': 2.0,
            'goal_timeout_seconds': 120.0,
            'max_retries': 2,
            'retry_delay_seconds': 2.0,
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
            'max_linear_speed': 0.24,
            'max_angular_speed': 0.6,
            'manual_command_timeout': 0.5,
            'camera_enabled_at_start': ParameterValue(
                use_gazebo,
                value_type=bool,
            ),
            'perception_initial_mode': 'fusion',
            # Gazebo publishes world-referenced odometry. The lightweight
            # simulator integrates from a local zero and needs the spawn offset.
            'odom_pose_is_world': ParameterValue(
                use_gazebo,
                value_type=bool,
            ),
            # A live SLAM session has no valid patrol route until the operator
            # saves a map and applies its edited route from Map Management.
            'patrol_route_ready_at_start': False,
            'autonomous_exploration_available': ParameterValue(
                enable_autonomous_exploration,
                value_type=bool,
            ),
            'require_3d_map_on_save': ParameterValue(
                enable_3d_mapping,
                value_type=bool,
            ),
        }],
    )

    ekf_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", use_gazebo, "' == 'true' and '",
            ground_truth_odometry, "' != 'true'",
        ])),
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

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(start_rviz),
        arguments=[
            '-d',
            PathJoinSubstitution(
                [
                    FindPackageShare('patrol_robot_navigation'),
                    'config',
                    'mapping_3d.rviz',
                ]
            ),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    default_map = PathJoinSubstitution([
        FindPackageShare('patrol_robot_navigation'),
        'maps',
        'pipeline_map.yaml',
    ])
    default_waypoints = PathJoinSubstitution([
        FindPackageShare('patrol_robot_patrol'),
        'config',
        'waypoints.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('use_gazebo', default_value='false'),
        DeclareLaunchArgument(
            'enable_3d_mapping',
            default_value='false',
            description='Build a persistent colored OctoMap from RGB-D points',
        ),
        DeclareLaunchArgument(
            'enable_autonomous_exploration',
            default_value='true',
            description='Start the Frontier Explorer and Nav2 services',
        ),
        DeclareLaunchArgument(
            'autonomous_exploration_autostart',
            default_value='false',
            description='Begin exploring immediately instead of waiting for the web button',
        ),
        DeclareLaunchArgument(
            'patrol_autostart',
            default_value='false',
            description='Wait for the web patrol button after a route is applied',
        ),
        DeclareLaunchArgument('patrol_loop', default_value='true'),
        DeclareLaunchArgument('waypoints', default_value=default_waypoints),
        DeclareLaunchArgument(
            'ground_truth_odometry',
            default_value='true',
            description='Use simulator truth instead of wheel odom + IMU EKF',
        ),
        DeclareLaunchArgument('map', default_value=default_map),
        gazebo_simulation,
        camera_processing,
        lightweight_simulation,
        ekf_localization,
        map_source_mux,
        mapping,
        static_map_server,
        static_amcl,
        static_localization_manager,
        octomap_mapping,
        exploration_navigation,
        frontier_explorer,
        navigation_health,
        patrol_manager,
        web_bridge,
        rviz,
    ])

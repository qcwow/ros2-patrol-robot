"""AMCL/Nav2/patrol bringup for the physical ROSOrin car."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
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
    start_hardware = LaunchConfiguration('start_hardware')
    start_rviz = LaunchConfiguration('start_rviz')
    start_web_bridge = LaunchConfiguration('start_web_bridge')
    start_patrol_manager = LaunchConfiguration('start_patrol_manager')
    map_yaml = LaunchConfiguration('map')
    waypoints = LaunchConfiguration('waypoints')
    patrol_autostart = LaunchConfiguration('patrol_autostart')
    web_port = LaunchConfiguration('web_port')
    navigation_share = FindPackageShare('patrol_robot_navigation')
    patrol_share = FindPackageShare('patrol_robot_patrol')
    camera_share = FindPackageShare('patrol_robot_camera')

    # Project-owned adapter: vendor code is limited to physical device drivers,
    # sensor calibration, encoder odometry, and the real robot description.
    hardware_adapter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('patrol_robot_bringup'),
            'launch',
            'real_car_hardware.launch.py',
        ])),
        condition=IfCondition(start_hardware),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            navigation_share,
            'launch',
            'navigation.launch.py',
        ])),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
            'map': map_yaml,
            'params_file': PathJoinSubstitution([
                navigation_share,
                'config',
                'nav2_params_real_car.yaml',
            ]),
            'ground_truth_localization': 'false',
        }.items(),
    )

    # Web/manual and smoothed Nav2 commands are arbitrated here, but the real
    # car output is a raw channel.  It cannot reach the controller directly.
    web_bridge = Node(
        package='patrol_robot_web_bridge',
        executable='web_bridge',
        name='patrol_robot_web_bridge',
        output='screen',
        condition=IfCondition(start_web_bridge),
        parameters=[{
            'use_sim_time': False,
            'http_host': '0.0.0.0',
            'http_port': ParameterValue(web_port, value_type=int),
            'max_linear_speed': 0.15,
            'max_angular_speed': 0.45,
            'speed_control_min_linear': 0.05,
            'speed_control_max_linear': 0.15,
            'speed_control_max_angular': 0.45,
            'use_patrol_speed_limits': False,
            'manual_command_timeout': 0.35,
            'base_command_topic': '/cmd_vel_base_raw',
            'scan_topic': '/scan_raw',
            'camera_topic': '/depth_cam/rgb0/image_raw',
            'camera_enabled_at_start': True,
            'perception_initial_mode': 'fusion',
            'ground_truth_localization': False,
            'seed_initial_pose_at_start': False,
            'patrol_route_ready_at_start': False,
            'autonomous_exploration_available': False,
        }],
    )

    safety = Node(
        package='patrol_robot_patrol',
        executable='manual_lidar_safety',
        name='manual_lidar_safety',
        output='screen',
        respawn=True,
        respawn_delay=0.2,
        parameters=[
            PathJoinSubstitution([
                navigation_share,
                'config',
                'manual_lidar_safety_real_car.yaml',
            ]),
            {
                'input_cmd_vel_topic': ParameterValue(
                    PythonExpression([
                        "'/cmd_vel_base_raw' if '",
                        start_web_bridge,
                        "' == 'true' else '/cmd_vel_nav'",
                    ]),
                    value_type=str,
                ),
            },
        ],
    )

    camera_processing = Node(
        package='patrol_robot_camera',
        executable='rgbd_processor',
        name='rgbd_processor',
        output='screen',
        parameters=[PathJoinSubstitution([
            camera_share,
            'config',
            'rgbd_processor_real_car.yaml',
        ])],
    )

    rgbd_obstacles = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            navigation_share,
            'launch',
            'rgbd_obstacles.launch.py',
        ])),
    )

    base_watchdog = Node(
        package='patrol_robot_patrol',
        executable='base_command_watchdog',
        name='base_command_watchdog',
        output='screen',
        respawn=True,
        respawn_delay=0.2,
        parameters=[{
            'use_sim_time': False,
            'input_cmd_vel_topic': '/cmd_vel_safety_checked',
            'output_cmd_vel_topic': '/controller/cmd_vel',
            'command_timeout': 0.25,
            'max_linear_speed': 0.15,
            'max_angular_speed': 0.45,
        }],
    )

    navigation_health = Node(
        package='patrol_robot_patrol',
        executable='navigation_health_monitor',
        name='navigation_health',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'scan_timeout_seconds': 1.0,
            'odom_timeout_seconds': 1.0,
            'amcl_timeout_seconds': 2.0,
            'costmap_timeout_seconds': 3.0,
            'footprint_length': 0.31,
            'footprint_width': 0.26,
            'footprint_safety_margin': 0.01,
            'require_amcl_covariance': True,
            'max_position_variance': 0.25,
            'max_yaw_variance': 0.25,
            'require_base_driver_ready': False,
            'require_estop_released': False,
        }],
        remappings=[('/scan', '/scan_raw')],
    )

    patrol_manager = Node(
        package='patrol_robot_patrol',
        executable='patrol_manager',
        name='patrol_manager',
        output='screen',
        condition=IfCondition(start_patrol_manager),
        parameters=[{
            'use_sim_time': False,
            'waypoint_file': waypoints,
            'autostart': patrol_autostart,
            'loop': True,
            'loop_count': 1,
            'start_delay_seconds': 10.0,
            'goal_timeout_seconds': 120.0,
            'max_retries': 1,
            'max_health_recoveries': 2,
            'health_recovery_timeout_seconds': 8.0,
            'failed_path_start_clearance': 0.50,
            'failed_path_goal_clearance': 0.50,
            'failed_path_band_radius': 0.15,
            'failed_path_similarity_distance': 0.45,
            'max_similar_path_replans': 4,
            'max_route_failures': 4,
            'action_name': 'navigate_to_pose',
            'behavior_tree': PathJoinSubstitution([
                patrol_share,
                'behavior_trees',
                'navigate_to_pose_stable.xml',
            ]),
            'behavior_tree_no_spin': PathJoinSubstitution([
                patrol_share,
                'behavior_trees',
                'navigate_to_pose_no_spin.xml',
            ]),
            'behavior_tree_restricted': PathJoinSubstitution([
                patrol_share,
                'behavior_trees',
                'navigate_to_pose_restricted.xml',
            ]),
            'require_navigation_health': True,
            'enforce_required_sensors': True,
            'enable_motion_spin_guard': True,
            'motion_spin_window_seconds': 4.0,
            'motion_spin_max_translation': 0.08,
            'motion_spin_min_yaw_degrees': 20.0,
            'motion_spin_min_goal_progress': 0.02,
            # Web speed is the only global Nav2 speed limit on the real car.
            'publish_nav2_speed_limit_direct': False,
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
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'rviz',
                'nav2_default_view.rviz',
            ]),
        ],
        parameters=[{'use_sim_time': False}],
    )

    default_map = PathJoinSubstitution([
        navigation_share, 'maps', 'pipeline_map.yaml'])
    default_waypoints = PathJoinSubstitution([
        patrol_share, 'config', 'waypoints_real_car_template.yaml'])

    return LaunchDescription([
        # Match the vendor boot environment; some ROSOrin installed launch
        # files do not support their need_compile=True branch.
        SetEnvironmentVariable('need_compile', 'False'),
        DeclareLaunchArgument('start_hardware', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('start_web_bridge', default_value='true'),
        DeclareLaunchArgument('start_patrol_manager', default_value='true'),
        DeclareLaunchArgument('patrol_autostart', default_value='false'),
        DeclareLaunchArgument('web_port', default_value='8765'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('waypoints', default_value=default_waypoints),
        hardware_adapter,
        navigation,
        web_bridge,
        base_watchdog,
        safety,
        camera_processing,
        rgbd_obstacles,
        navigation_health,
        patrol_manager,
        rviz,
    ])

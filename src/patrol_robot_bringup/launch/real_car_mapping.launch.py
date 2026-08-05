"""Unified ROSOrin live SLAM, Nav2, patrol, and safe web control."""

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
    start_joystick = LaunchConfiguration('start_joystick')
    start_web_bridge = LaunchConfiguration('start_web_bridge')
    start_rviz = LaunchConfiguration('start_rviz')
    start_local_grid = LaunchConfiguration('start_local_grid')
    start_rotation_diagnostics = LaunchConfiguration(
        'start_rotation_diagnostics')
    start_octomap = LaunchConfiguration('start_octomap')
    mapping_backend = LaunchConfiguration('mapping_backend')
    reset_rtabmap_database = LaunchConfiguration('reset_rtabmap_database')
    rtabmap_database_path = LaunchConfiguration('rtabmap_database_path')
    rtabmap_params_file = LaunchConfiguration('rtabmap_params_file')
    rtabmap_imu_topic = LaunchConfiguration('rtabmap_imu_topic')
    rtabmap_rgb_topic = LaunchConfiguration('rtabmap_rgb_topic')
    rtabmap_depth_topic = LaunchConfiguration('rtabmap_depth_topic')
    rtabmap_camera_info_topic = LaunchConfiguration(
        'rtabmap_camera_info_topic')
    web_port = LaunchConfiguration('web_port')
    max_linear = LaunchConfiguration('max_linear_speed')
    max_angular = LaunchConfiguration('max_angular_speed')
    waypoints = LaunchConfiguration('waypoints')
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

    safe_joystick = Node(
        package='patrol_robot_patrol',
        executable='safe_joystick_teleop',
        name='safe_joystick_teleop',
        output='screen',
        condition=IfCondition(start_joystick),
        parameters=[{
            'use_sim_time': False,
            'joy_topic': '/ros_robot_controller/joy',
            'output_cmd_vel_topic': '/cmd_vel_manual_raw',
            'max_linear_speed': ParameterValue(max_linear, value_type=float),
            'max_angular_speed': ParameterValue(max_angular, value_type=float),
        }],
    )

    # The bridge arbitrates live-SLAM Nav2 and manual commands. Its output is
    # still a raw channel which cannot reach the chassis without lidar safety.
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
            'max_linear_speed': ParameterValue(max_linear, value_type=float),
            'max_angular_speed': ParameterValue(max_angular, value_type=float),
            'speed_control_min_linear': 0.05,
            'speed_control_max_linear': ParameterValue(
                max_linear, value_type=float),
            'speed_control_max_angular': ParameterValue(
                max_angular, value_type=float),
            'use_patrol_speed_limits': False,
            'manual_command_timeout': 0.35,
            'base_command_topic': '/cmd_vel_base_raw',
            'scan_topic': '/scan_raw',
            'camera_topic': '/depth_cam/rgb0/image_raw',
            'camera_enabled_at_start': True,
            # Keep the operator view usable without letting JPEG encoding
            # compete with RGB-D mapping and Nav2 on the Orin.
            'camera_stream_fps': 6.0,
            'camera_stream_width': 480,
            'camera_jpeg_quality': 55,
            'perception_initial_mode': 'fusion',
            'ground_truth_localization': False,
            'seed_initial_pose_at_start': False,
            'patrol_route_ready_at_start': False,
            'autonomous_exploration_available': True,
            'mapping_backend': mapping_backend,
            'require_3d_map_on_save': True,
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
                # With the web bridge running, it arbitrates Nav2/manual input
                # onto /cmd_vel_base_raw.  Headless mapping has no bridge, so
                # feed the smoothed Nav2 channel into the same lidar filter.
                'input_cmd_vel_topic': ParameterValue(
                    PythonExpression([
                        "'/cmd_vel_base_raw' if '",
                        start_web_bridge,
                        "' == 'true' else '/cmd_vel_nav'",
                    ]),
                    value_type=str,
                ),
                'max_linear_speed': ParameterValue(
                    max_linear, value_type=float),
                'max_angular_speed': ParameterValue(
                    max_angular, value_type=float),
            },
        ],
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
            'max_linear_speed': ParameterValue(max_linear, value_type=float),
            'max_angular_speed': ParameterValue(max_angular, value_type=float),
        }],
    )

    slam_toolbox_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            navigation_share,
            'launch',
            'mapping.launch.py',
        ])),
        condition=IfCondition(PythonExpression([
            "'", mapping_backend, "' == 'slam_toolbox'",
        ])),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': PathJoinSubstitution([
                navigation_share,
                'config',
                'slam_toolbox_real_car.yaml',
            ]),
            'map_topic': '/map',
        }.items(),
    )

    rtabmap_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            navigation_share,
            'launch',
            'rtabmap_mapping.launch.py',
        ])),
        condition=IfCondition(PythonExpression([
            "'", mapping_backend, "' == 'rtabmap'",
        ])),
        launch_arguments={
            'use_sim_time': 'false',
            'reset_database': reset_rtabmap_database,
            'database_path': rtabmap_database_path,
            'rtabmap_params_file': rtabmap_params_file,
            'rgb_topic': rtabmap_rgb_topic,
            'depth_topic': rtabmap_depth_topic,
            'camera_info_topic': rtabmap_camera_info_topic,
            'scan_topic': '/scan_raw',
            'odom_topic': '/odom',
            'imu_topic': rtabmap_imu_topic,
            'map_topic': '/map',
        }.items(),
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

    # Robot-centred RGB-D voxel map. This is the ROS 2 counterpart of the
    # reference project's T265/D435 ring buffer; persistent global 3D mapping
    # remains the responsibility of OctoMap below.
    local_grid = Node(
        package='patrol_robot_camera',
        executable='local_voxel_mapper',
        name='local_voxel_mapper',
        output='screen',
        condition=IfCondition(start_local_grid),
        parameters=[PathJoinSubstitution([
            camera_share,
            'config',
            'local_voxel_mapper_real_car.yaml',
        ])],
    )

    octomap_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            navigation_share,
            'launch',
            'octomap_mapping.launch.py',
        ])),
        condition=IfCondition(PythonExpression([
            "'", mapping_backend, "' == 'slam_toolbox' and '",
            start_octomap,
            "' == 'true'",
        ])),
        launch_arguments={
            'use_sim_time': 'false',
            'octomap_params_file': PathJoinSubstitution([
                navigation_share,
                'config',
                'octomap_server_real_car.yaml',
            ]),
            'octomap_point_cloud_topic': '/camera/points/mapping',
            'octomap_server_executable': 'octomap_server_node',
        }.items(),
    )

    live_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            navigation_share,
            'launch',
            'exploration_navigation.launch.py',
        ])),
        launch_arguments={
            'use_sim_time': 'false',
            'exploration_nav2_params_file': PathJoinSubstitution([
                navigation_share,
                'config',
                'nav2_params_real_car.yaml',
            ]),
            'desired_linear_vel': max_linear,
            # The real-car behavior tree contains no recovery actions; omit
            # the server instead of configuring an untyped empty plugin list.
            'enable_behavior_server': 'false',
        }.items(),
    )

    frontier_explorer = Node(
        package='patrol_robot_patrol',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': False,
            'start_delay_seconds': 2.0,
            'planning_period_seconds': 2.5,
            'goal_timeout_seconds': 120.0,
            'max_unreachable_cycles': 5,
            'robot_clearance': 0.34,
            'stop_command_topic': '/cmd_vel_nav_raw',
            'behavior_tree': PathJoinSubstitution([
                patrol_share,
                'behavior_trees',
                'navigate_to_frontier.xml',
            ]),
            'require_navigation_health': True,
            'enable_motion_spin_guard': True,
            'motion_spin_window_seconds': 4.0,
            'motion_spin_max_translation': 0.08,
            'motion_spin_min_yaw_degrees': 20.0,
            'motion_spin_min_goal_progress': 0.02,
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
            'costmap_timeout_seconds': 5.0,
            'footprint_length': 0.31,
            'footprint_width': 0.26,
            'footprint_safety_margin': 0.01,
            # The selected mapping backend, not AMCL, owns map -> odom.
            'require_amcl_covariance': False,
            # RTAB-Map may adjust map -> odom while its first RGB-D/scan nodes
            # settle. Do not release patrol until that transform has remained
            # within the measured real-car bounds for a full window.
            'require_global_pose_stability': True,
            'global_pose_stable_seconds': 5.0,
            'global_pose_max_translation_delta': 0.08,
            'global_pose_max_yaw_delta_degrees': 5.0,
            'require_base_driver_ready': False,
            'require_estop_released': False,
            'lifecycle_nodes': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'velocity_smoother',
                'bt_navigator',
            ],
        }],
        remappings=[('/scan', '/scan_raw')],
    )

    rotation_diagnostics = Node(
        package='patrol_robot_patrol',
        executable='rotation_diagnostic_recorder',
        name='rotation_diagnostic_recorder',
        output='screen',
        condition=IfCondition(start_rotation_diagnostics),
        parameters=[{
            'use_sim_time': False,
            'pre_event_seconds': 10.0,
            'post_event_seconds': 2.0,
            'sample_period_seconds': 0.1,
        }],
    )

    patrol_manager = Node(
        package='patrol_robot_patrol',
        executable='patrol_manager',
        name='patrol_manager',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'waypoint_file': waypoints,
            'autostart': False,
            'loop': True,
            'loop_count': 1,
            'start_delay_seconds': 2.0,
            'goal_timeout_seconds': 120.0,
            'max_retries': 1,
            'max_health_recoveries': 2,
            'health_recovery_timeout_seconds': 8.0,
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
            # The web bridge is the only Nav2 SpeedLimit publisher in the
            # real-car profile; waypoint roles never override web speed.
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
                navigation_share,
                'config',
                'mapping_3d.rviz',
            ]),
        ],
        parameters=[{'use_sim_time': False}],
        remappings=[('/scan', '/scan_raw')],
    )

    return LaunchDescription([
        # ROSOrin's vendor launch files use their source-tree configuration
        # when this is False.  That is also the factory boot configuration.
        SetEnvironmentVariable('need_compile', 'False'),
        DeclareLaunchArgument('start_hardware', default_value='true'),
        DeclareLaunchArgument('start_joystick', default_value='false'),
        DeclareLaunchArgument('start_web_bridge', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument(
            'start_local_grid',
            default_value='false',
            description=(
                'Optional 3D preview only; disabled by default on the real '
                'car to preserve CPU for mapping, TF, Nav2, and safety'
            ),
        ),
        DeclareLaunchArgument(
            'start_rotation_diagnostics',
            default_value='false',
            description=(
                'Enable the resource-intensive rotation evidence recorder '
                'only during a targeted diagnostic run'
            ),
        ),
        DeclareLaunchArgument(
            'mapping_backend',
            default_value='rtabmap',
            choices=['rtabmap', 'slam_toolbox'],
            description='rtabmap or slam_toolbox; never start both',
        ),
        DeclareLaunchArgument(
            'reset_rtabmap_database',
            default_value='true',
            description='Delete the RTAB-Map database when starting a new map',
        ),
        DeclareLaunchArgument(
            'rtabmap_database_path',
            default_value='~/.ros/rtabmap.db',
        ),
        DeclareLaunchArgument(
            'rtabmap_params_file',
            default_value=PathJoinSubstitution([
                navigation_share,
                'config',
                'rtabmap_real_car.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'rtabmap_imu_topic',
            default_value='/rtabmap/imu_disabled',
            description=(
                'RTAB-Map direct IMU input; fused /odom remains enabled'
            ),
        ),
        DeclareLaunchArgument(
            'rtabmap_rgb_topic',
            default_value='/depth_cam/rgb0/image_raw',
        ),
        DeclareLaunchArgument(
            'rtabmap_depth_topic',
            default_value='/depth_cam/depth0/image_raw',
        ),
        DeclareLaunchArgument(
            'rtabmap_camera_info_topic',
            default_value='/depth_cam/rgb0/camera_info',
        ),
        DeclareLaunchArgument(
            'start_octomap',
            default_value='true',
            description='Only used by the slam_toolbox fallback backend',
        ),
        DeclareLaunchArgument('web_port', default_value='8765'),
        DeclareLaunchArgument(
            'waypoints',
            default_value=PathJoinSubstitution([
                patrol_share,
                'config',
                'waypoints_real_car_template.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'max_linear_speed',
            default_value='0.15',
            description='Initial real-car mapping limit in m/s',
        ),
        DeclareLaunchArgument(
            'max_angular_speed',
            default_value='0.45',
            description='Initial real-car mapping yaw limit in rad/s',
        ),
        hardware_adapter,
        base_watchdog,
        safety,
        web_bridge,
        safe_joystick,
        slam_toolbox_mapping,
        rtabmap_mapping,
        camera_processing,
        rgbd_obstacles,
        local_grid,
        octomap_mapping,
        live_navigation,
        frontier_explorer,
        navigation_health,
        rotation_diagnostics,
        patrol_manager,
        rviz,
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def nav2_node(package, executable, name, params_file, use_sim_time,
              remappings=None, extra_parameters=None):
    return Node(
        package=package,
        executable=executable,
        name=name,
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time, **(extra_parameters or {})},
        ],
        remappings=remappings or [],
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    ground_truth_localization = LaunchConfiguration('ground_truth_localization')

    navigation_share = FindPackageShare('patrol_robot_navigation')
    default_map = PathJoinSubstitution(
        [navigation_share, 'maps', 'pipeline_map.yaml']
    )
    default_params = PathJoinSubstitution(
        [navigation_share, 'config', 'nav2_params.yaml']
    )

    tf_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {'yaml_filename': map_yaml, 'use_sim_time': use_sim_time},
        ],
        remappings=tf_remappings,
    )
    amcl = nav2_node(
        'nav2_amcl', 'amcl', 'amcl', params_file, use_sim_time,
        tf_remappings,
        {
            # Gazebo already provides an exact world pose. AMCL may jump
            # between similar dense obstacles, so it can estimate particles
            # for diagnostics but must not override map->odom in that mode.
            'tf_broadcast': ParameterValue(
                PythonExpression([
                    "'", ground_truth_localization, "' != 'true'",
                ]),
                value_type=bool,
            ),
        },
    )

    localization_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    controller = nav2_node(
        'nav2_controller',
        'controller_server',
        'controller_server',
        params_file,
        use_sim_time,
        # Controllers and recovery behaviors share a raw navigation channel.
        # Only the velocity smoother may publish the final navigation command.
        tf_remappings + [('cmd_vel', 'cmd_vel_nav_raw')],
    )
    smoother = nav2_node(
        'nav2_smoother',
        'smoother_server',
        'smoother_server',
        params_file,
        use_sim_time,
        tf_remappings + [('cmd_vel', 'cmd_vel_nav_raw')],
    )
    planner = nav2_node(
        'nav2_planner',
        'planner_server',
        'planner_server',
        params_file,
        use_sim_time,
        tf_remappings,
    )
    behaviors = nav2_node(
        'nav2_behaviors',
        'behavior_server',
        'behavior_server',
        params_file,
        use_sim_time,
        # Recovery commands must pass through the same smoother and manual
        # arbitration path as ordinary controller commands. Nothing except the
        # web bridge may publish directly to the physical /cmd_vel topic.
        tf_remappings + [('cmd_vel', 'cmd_vel_nav_raw')],
    )
    bt_navigator = nav2_node(
        'nav2_bt_navigator',
        'bt_navigator',
        'bt_navigator',
        params_file,
        use_sim_time,
        tf_remappings,
    )
    waypoint_follower = nav2_node(
        'nav2_waypoint_follower',
        'waypoint_follower',
        'waypoint_follower',
        params_file,
        use_sim_time,
        tf_remappings,
    )
    velocity_smoother = nav2_node(
        'nav2_velocity_smoother',
        'velocity_smoother',
        'velocity_smoother',
        params_file,
        use_sim_time,
        tf_remappings + [
            ('cmd_vel', 'cmd_vel_nav_raw'),
            ('cmd_vel_smoothed', 'cmd_vel_nav'),
        ],
    )

    navigation_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'velocity_smoother',
                'bt_navigator',
                'waypoint_follower',
            ],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('ground_truth_localization', default_value='true'),
        map_server,
        amcl,
        localization_manager,
        controller,
        smoother,
        planner,
        behaviors,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        navigation_manager,
    ])

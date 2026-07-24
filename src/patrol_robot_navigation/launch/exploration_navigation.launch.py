"""Nav2 motion stack for live SLAM exploration without AMCL or map_server."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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
    autostart = LaunchConfiguration('exploration_nav2_autostart')
    params_file = LaunchConfiguration('exploration_nav2_params_file')
    navigation_share = FindPackageShare('patrol_robot_navigation')
    default_params = PathJoinSubstitution([
        navigation_share,
        'config',
        'nav2_params.yaml',
    ])
    tf_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    controller = nav2_node(
        'nav2_controller',
        'controller_server',
        'controller_server',
        params_file,
        use_sim_time,
        tf_remappings + [('cmd_vel', 'cmd_vel_nav_raw')],
        {
            # Exploration favors observability and braking distance over
            # transit speed while the global map is still changing.
            'FollowPath.desired_linear_vel': 0.24,
        },
    )
    smoother = nav2_node(
        'nav2_smoother',
        'smoother_server',
        'smoother_server',
        params_file,
        use_sim_time,
        tf_remappings,
    )
    planner = nav2_node(
        'nav2_planner',
        'planner_server',
        'planner_server',
        params_file,
        use_sim_time,
        tf_remappings,
        {
            # A frontier goal is placed on known free space immediately behind
            # the unknown boundary. The route to it must stay in observed free
            # space instead of cutting through unmapped cells.
            'GridBased.allow_unknown': False,
        },
    )
    behaviors = nav2_node(
        'nav2_behaviors',
        'behavior_server',
        'behavior_server',
        params_file,
        use_sim_time,
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
    velocity_smoother = nav2_node(
        'nav2_velocity_smoother',
        'velocity_smoother',
        'velocity_smoother',
        params_file,
        use_sim_time,
        tf_remappings + [
            ('cmd_vel', 'cmd_vel_nav_raw'),
            ('cmd_vel_smoothed', '/cmd_vel_nav'),
        ],
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_exploration_navigation',
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
            ],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'exploration_nav2_autostart',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'exploration_nav2_params_file',
            default_value=default_params,
        ),
        controller,
        smoother,
        planner,
        behaviors,
        bt_navigator,
        velocity_smoother,
        lifecycle_manager,
    ])

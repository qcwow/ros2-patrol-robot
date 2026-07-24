from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    # These names must stay unique across included launch descriptions.
    # simulation_mapping.launch.py also includes the camera and SLAM launch
    # files, whose generic params_file / point_cloud_topic arguments otherwise
    # leak into this include and silently reconfigure OctoMap.
    octomap_params_file = LaunchConfiguration('octomap_params_file')
    octomap_point_cloud_topic = LaunchConfiguration(
        'octomap_point_cloud_topic'
    )
    navigation_share = FindPackageShare('patrol_robot_navigation')

    default_params = PathJoinSubstitution([
        navigation_share,
        'config',
        'octomap_server.yaml',
    ])

    octomap_server = Node(
        package='octomap_server',
        executable='color_octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[
            octomap_params_file,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('cloud_in', octomap_point_cloud_topic),
            ('projected_map', '/octomap/projected_map'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'octomap_params_file',
            default_value=default_params,
        ),
        DeclareLaunchArgument(
            'octomap_point_cloud_topic',
            default_value='/camera/points/mapping',
        ),
        octomap_server,
    ])

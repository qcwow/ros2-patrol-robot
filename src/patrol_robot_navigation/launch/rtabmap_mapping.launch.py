"""RTAB-Map RGB-D + 2D lidar mapping for the physical ROSOrin car."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def rtabmap_node(condition, arguments):
    return Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        condition=condition,
        parameters=[
            LaunchConfiguration('rtabmap_params_file'),
            {
                'database_path': LaunchConfiguration('database_path'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
        remappings=[
            ('rgbd_image', '/rtabmap/rgbd_image'),
            ('scan', LaunchConfiguration('scan_topic')),
            ('odom', LaunchConfiguration('odom_topic')),
            ('imu', LaunchConfiguration('imu_topic')),
            ('map', LaunchConfiguration('map_topic')),
            ('cloud_map', '/rtabmap/cloud_map'),
            ('cloud_obstacles', '/rtabmap/cloud_obstacles'),
            ('cloud_ground', '/rtabmap/cloud_ground'),
            # Keep the conventional names so the existing 3D map saver can
            # persist RTAB-Map's built-in OctoMap without another mapper.
            ('octomap_binary', '/octomap_binary'),
            ('octomap_full', '/octomap_full'),
        ],
        arguments=arguments,
    )


def generate_launch_description():
    navigation_share = FindPackageShare('patrol_robot_navigation')
    default_params = PathJoinSubstitution([
        navigation_share,
        'config',
        'rtabmap_real_car.yaml',
    ])

    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'approx_sync': True,
            'approx_sync_max_interval': 0.10,
            'queue_size': 10,
            'qos': 2,
            'qos_camera_info': 2,
        }],
        remappings=[
            ('rgb/image', LaunchConfiguration('rgb_topic')),
            ('rgb/camera_info', LaunchConfiguration('camera_info_topic')),
            ('depth/image', LaunchConfiguration('depth_topic')),
            ('rgbd_image', '/rtabmap/rgbd_image'),
        ],
    )

    clean_start = rtabmap_node(
        IfCondition(LaunchConfiguration('reset_database')),
        ['-d'],
    )
    resume = rtabmap_node(
        UnlessCondition(LaunchConfiguration('reset_database')),
        [],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('reset_database', default_value='true'),
        DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap.db'),
        DeclareLaunchArgument(
            'rtabmap_params_file', default_value=default_params),
        DeclareLaunchArgument('rgb_topic', default_value='/depth_cam/rgb0/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/depth_cam/depth0/image_raw'),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/depth_cam/rgb0/camera_info',
        ),
        DeclareLaunchArgument('scan_topic', default_value='/scan_raw'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/rtabmap/imu_disabled',
            description=(
                'Direct RTAB-Map IMU input. Disabled by default because the '
                'real-car EKF already fuses IMU into /odom.'
            ),
        ),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        rgbd_sync,
        clean_start,
        resume,
    ])

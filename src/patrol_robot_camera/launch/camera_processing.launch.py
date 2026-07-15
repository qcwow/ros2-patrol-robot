from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')
    color_topic = LaunchConfiguration('color_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    point_cloud_topic = LaunchConfiguration('point_cloud_topic')

    default_config = PathJoinSubstitution([
        FindPackageShare('patrol_robot_camera'),
        'config',
        'rgbd_processor.yaml',
    ])

    processor = Node(
        package='patrol_robot_camera',
        executable='rgbd_processor',
        name='rgbd_processor',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': use_sim_time,
                'color_topic': color_topic,
                'depth_topic': depth_topic,
                'camera_info_topic': camera_info_topic,
                'point_cloud_topic': point_cloud_topic,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument(
            'color_topic',
            default_value='/camera/color/image_raw',
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/depth/image_rect_raw',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/depth/camera_info',
        ),
        DeclareLaunchArgument(
            'point_cloud_topic',
            default_value='/camera/points/filtered',
        ),
        processor,
    ])

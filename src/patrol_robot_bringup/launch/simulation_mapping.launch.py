from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    start_rviz = LaunchConfiguration('start_rviz')
    use_gazebo = LaunchConfiguration('use_gazebo')
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
        }.items(),
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

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('patrol_robot_navigation'), 'launch', 'mapping.launch.py']
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
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

    default_map = PathJoinSubstitution([
        FindPackageShare('patrol_robot_navigation'),
        'maps',
        'pipeline_map.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('use_gazebo', default_value='false'),
        DeclareLaunchArgument('map', default_value=default_map),
        gazebo_simulation,
        lightweight_simulation,
        mapping,
        rviz,
    ])

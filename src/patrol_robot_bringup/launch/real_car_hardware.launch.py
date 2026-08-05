"""Minimal ROSOrin hardware adapter for the physical patrol robot.

Vendor packages are intentionally limited to device drivers, robot geometry,
and sensor calibration.  Navigation, mapping, patrol, teleoperation, safety,
and command arbitration belong to the patrol_robot workspace.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _vendor_launch(package, filename, launch_arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package),
            'launch',
            filename,
        ])),
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    bringup_share = FindPackageShare('patrol_robot_bringup')

    # These narrow vendor launches are the hardware adaptation boundary:
    # chassis serial driver + encoder odometry + physical robot TF, calibrated
    # IMU, lidar driver, and RGB-D camera driver.  In particular, do not use
    # the vendor robot.launch.py/controller.launch.py application stacks here.
    chassis_and_odometry = _vendor_launch(
        'controller', 'odom_publisher.launch.py')
    imu_driver = _vendor_launch('peripherals', 'imu_filter.launch.py')
    lidar_driver = _vendor_launch('peripherals', 'lidar.launch.py')
    rgbd_camera_driver = _vendor_launch(
        'peripherals',
        'depth_camera.launch.py',
        {
            # The project builds its own bounded/filtered cloud from the
            # depth image.  Avoid paying for the vendor's unused raw cloud.
            'point_cloud_enable': 'false',
            # RTAB-Map's rgbd_sync expects depth pixels registered to RGB.
            'align_mode': 'true',
            'depth_correction': 'true',
        },
    )

    # State estimation is project-owned.  The inputs remain the calibrated
    # hardware topics exposed by the adapter above.
    state_estimator = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[PathJoinSubstitution([
            bringup_share,
            'config',
            'ekf_real_car.yaml',
        ])],
        remappings=[('odometry/filtered', 'odom')],
    )

    return LaunchDescription([
        chassis_and_odometry,
        imu_driver,
        lidar_driver,
        rgbd_camera_driver,
        state_estimator,
    ])

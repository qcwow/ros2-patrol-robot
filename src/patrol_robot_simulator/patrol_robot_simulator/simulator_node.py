import base64
import json
import math
import time
from pathlib import Path

import rclpy
import yaml
from builtin_interfaces.msg import Time as TimeMessage
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_from_yaw(yaw: float) -> tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class OccupancyMap:
    def __init__(self, yaml_path: str) -> None:
        self.yaml_path = Path(yaml_path).expanduser().resolve()
        if not self.yaml_path.is_file():
            raise FileNotFoundError(f'地图配置不存在: {self.yaml_path}')

        with self.yaml_path.open('r', encoding='utf-8') as stream:
            config = yaml.safe_load(stream) or {}

        image_path = Path(str(config['image']))
        if not image_path.is_absolute():
            image_path = self.yaml_path.parent / image_path

        self.resolution = float(config['resolution'])
        origin = config.get('origin', [0.0, 0.0, 0.0])
        self.origin_x = float(origin[0])
        self.origin_y = float(origin[1])
        self.origin_yaw = float(origin[2])
        self.negate = bool(config.get('negate', 0))
        self.occupied_threshold = float(config.get('occupied_thresh', 0.65))

        if abs(self.origin_yaw) > 1.0e-9:
            raise ValueError('轻量仿真器暂不支持旋转后的栅格地图原点')

        self.width, self.height, maximum, pixels = self._read_p2_pgm(image_path)
        self._occupied = []
        for pixel in pixels:
            normalized = pixel / maximum
            probability = normalized if self.negate else 1.0 - normalized
            self._occupied.append(probability >= self.occupied_threshold)

    @staticmethod
    def _read_p2_pgm(path: Path) -> tuple[int, int, int, list[int]]:
        tokens = []
        for line in path.read_text(encoding='ascii').splitlines():
            content = line.split('#', 1)[0].strip()
            if content:
                tokens.extend(content.split())

        if not tokens or tokens[0] != 'P2':
            raise ValueError(f'轻量仿真器当前只支持 ASCII P2 PGM地图: {path}')
        width, height, maximum = map(int, tokens[1:4])
        pixels = list(map(int, tokens[4:]))
        if len(pixels) != width * height:
            raise ValueError(
                f'地图像素数量错误: 期望 {width * height}，实际 {len(pixels)}'
            )
        return width, height, maximum, pixels

    def is_occupied(self, world_x: float, world_y: float) -> bool:
        grid_x = int(math.floor((world_x - self.origin_x) / self.resolution))
        grid_y = int(math.floor((world_y - self.origin_y) / self.resolution))
        if grid_x < 0 or grid_y < 0:
            return True
        if grid_x >= self.width or grid_y >= self.height:
            return True

        image_row = self.height - 1 - grid_y
        return self._occupied[image_row * self.width + grid_x]

    @classmethod
    def from_scenario(cls, payload: dict):
        bounds = payload.get('bounds') or {}
        origin_x = float(bounds.get('minX', -8.0))
        origin_y = float(bounds.get('minY', -6.0))
        world_width = max(2.0, min(float(bounds.get('width', 16.0)), 500.0))
        world_height = max(2.0, min(float(bounds.get('height', 12.0)), 500.0))
        requested_resolution = max(
            0.02,
            min(float(payload.get('resolution', 0.25)), 5.0),
        )
        occupancy = payload.get('occupancy')
        if isinstance(occupancy, dict) and occupancy.get('data'):
            width = max(1, min(int(occupancy.get('width', 0)), 2000))
            height = max(1, min(int(occupancy.get('height', 0)), 2000))
            resolution = max(
                0.001,
                min(float(occupancy.get('resolution', requested_resolution)), 5.0),
            )
            origin_x = float(occupancy.get('originX', origin_x))
            origin_y = float(occupancy.get('originY', origin_y))
            packed = base64.b64decode(str(occupancy['data']), validate=True)
            if len(packed) * 8 < width * height:
                raise ValueError('占用栅格数据不完整')
            occupied = [
                bool((packed[index >> 3] >> (index & 7)) & 1)
                for index in range(width * height)
            ]
        else:
            resolution = max(
                requested_resolution,
                world_width / 800.0,
                world_height / 800.0,
            )
            width = max(2, int(math.ceil(world_width / resolution)))
            height = max(2, int(math.ceil(world_height / resolution)))
            occupied = [False] * (width * height)
            for row in range(height):
                for column in range(width):
                    if row in (0, height - 1) or column in (0, width - 1):
                        occupied[row * width + column] = True

        objects = payload.get('objects') or []
        if not isinstance(objects, list) or len(objects) > 500:
            raise ValueError('场景元素格式无效或数量超过 500')
        for item in objects:
            if not isinstance(item, dict):
                continue
            center_x = float(item.get('x', 0.0))
            center_y = float(item.get('y', 0.0))
            object_width = max(0.1, min(float(item.get('width', 1.0)), 50.0))
            object_depth = max(0.1, min(float(item.get('depth', 1.0)), 50.0))
            min_column = max(0, int(math.floor(
                (center_x - object_width / 2.0 - origin_x) / resolution
            )))
            max_column = min(width - 1, int(math.floor(
                (center_x + object_width / 2.0 - origin_x) / resolution
            )))
            min_grid_y = max(0, int(math.floor(
                (center_y - object_depth / 2.0 - origin_y) / resolution
            )))
            max_grid_y = min(height - 1, int(math.floor(
                (center_y + object_depth / 2.0 - origin_y) / resolution
            )))
            for grid_y in range(min_grid_y, max_grid_y + 1):
                row = height - 1 - grid_y
                for column in range(min_column, max_column + 1):
                    occupied[row * width + column] = True

        instance = cls.__new__(cls)
        instance.yaml_path = Path('<web-scenario>')
        instance.resolution = resolution
        instance.origin_x = origin_x
        instance.origin_y = origin_y
        instance.origin_yaw = 0.0
        instance.negate = False
        instance.occupied_threshold = 0.65
        instance.width = width
        instance.height = height
        instance._occupied = occupied
        return instance


class LightweightSimulator(Node):
    def __init__(self) -> None:
        super().__init__('lightweight_simulator')

        self.declare_parameter('map_yaml', '')
        self.declare_parameter('initial_x', -6.0)
        self.declare_parameter('initial_y', -4.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('robot_radius', 0.30)
        self.declare_parameter('laser_offset_x', 0.12)
        self.declare_parameter('scan_samples', 360)
        self.declare_parameter('scan_rate', 10.0)
        self.declare_parameter('scan_min_range', 0.08)
        self.declare_parameter('scan_max_range', 12.0)
        self.declare_parameter('linear_speed_limit', 0.6)
        self.declare_parameter('angular_speed_limit', 1.8)
        self.declare_parameter('command_timeout', 0.5)

        map_yaml = str(self.get_parameter('map_yaml').value)
        if not map_yaml:
            raise ValueError('参数 map_yaml 不能为空')
        self._map = OccupancyMap(map_yaml)

        self._initial_x = float(self.get_parameter('initial_x').value)
        self._initial_y = float(self.get_parameter('initial_y').value)
        self._initial_yaw = float(self.get_parameter('initial_yaw').value)
        self._robot_radius = float(self.get_parameter('robot_radius').value)
        self._laser_offset_x = float(self.get_parameter('laser_offset_x').value)
        self._scan_samples = max(60, int(self.get_parameter('scan_samples').value))
        self._scan_rate = max(1.0, float(self.get_parameter('scan_rate').value))
        self._scan_min = float(self.get_parameter('scan_min_range').value)
        self._scan_max = float(self.get_parameter('scan_max_range').value)
        self._linear_limit = float(
            self.get_parameter('linear_speed_limit').value
        )
        self._angular_limit = float(
            self.get_parameter('angular_speed_limit').value
        )
        self._command_timeout = float(self.get_parameter('command_timeout').value)

        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._linear_command = 0.0
        self._angular_command = 0.0
        self._actual_linear = 0.0
        self._actual_angular = 0.0
        self._left_wheel_position = 0.0
        self._right_wheel_position = 0.0
        self._wheel_radius = 0.085
        self._wheel_separation = 0.39

        now = time.monotonic()
        self._wall_start = now
        self._last_update = now
        self._last_command = now
        self._last_scan = -1.0
        self._last_collision_log = -10.0

        self._clock_publisher = self.create_publisher(Clock, '/clock', 10)
        self._odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self._joint_publisher = self.create_publisher(
            JointState, '/joint_states', 10
        )
        self._imu_publisher = self.create_publisher(
            Imu, '/imu', qos_profile_sensor_data
        )
        self._scan_publisher = self.create_publisher(
            LaserScan, '/scan', qos_profile_sensor_data
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self._on_command, 10)
        self.create_subscription(
            String,
            '/patrol/map_scenario',
            self._on_map_scenario,
            10,
        )

        angle_increment = 2.0 * math.pi / (self._scan_samples - 1)
        self._scan_angles = [
            -math.pi + index * angle_increment
            for index in range(self._scan_samples)
        ]
        self._angle_increment = angle_increment
        self._ray_step = min(0.10, self._map.resolution / 2.0)

        self.create_timer(0.02, self._update)
        self.get_logger().info(
            f'轻量二维仿真器已启动：地图={self._map.width}x{self._map.height}，'
            f'初始位姿=({self._initial_x:.2f}, {self._initial_y:.2f}, '
            f'{self._initial_yaw:.2f})，激光={self._scan_samples}束'
        )

    def _on_command(self, message: Twist) -> None:
        self._linear_command = max(
            -self._linear_limit,
            min(self._linear_limit, float(message.linear.x)),
        )
        self._angular_command = max(
            -self._angular_limit,
            min(self._angular_limit, float(message.angular.z)),
        )
        self._last_command = time.monotonic()

    def _on_map_scenario(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            next_map = OccupancyMap.from_scenario(payload)
            if not next_map.is_occupied(*self._map_pose()[:2]):
                self._map = next_map
                self._ray_step = min(0.10, self._map.resolution / 2.0)
                self.get_logger().warning(
                    f'仿真场景已切换：{payload.get("name", "未命名地图")}，'
                    f'{next_map.width}x{next_map.height} 栅格'
                )
            else:
                self.get_logger().error('新地图中车辆当前位置被占用，拒绝切换仿真场景')
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as error:
            self.get_logger().error(f'地图场景数据无效：{error}')

    @staticmethod
    def _stamp(simulation_time: float) -> TimeMessage:
        seconds = int(simulation_time)
        nanoseconds = int((simulation_time - seconds) * 1_000_000_000)
        return TimeMessage(sec=seconds, nanosec=nanoseconds)

    def _map_pose(self, odom_x=None, odom_y=None, odom_yaw=None):
        x = self._odom_x if odom_x is None else odom_x
        y = self._odom_y if odom_y is None else odom_y
        yaw = self._odom_yaw if odom_yaw is None else odom_yaw
        cosine = math.cos(self._initial_yaw)
        sine = math.sin(self._initial_yaw)
        return (
            self._initial_x + cosine * x - sine * y,
            self._initial_y + sine * x + cosine * y,
            normalize_angle(self._initial_yaw + yaw),
        )

    def _pose_is_free(self, map_x: float, map_y: float) -> bool:
        if self._map.is_occupied(map_x, map_y):
            return False
        for index in range(16):
            angle = index * math.pi / 8.0
            check_x = map_x + self._robot_radius * math.cos(angle)
            check_y = map_y + self._robot_radius * math.sin(angle)
            if self._map.is_occupied(check_x, check_y):
                return False
        return True

    def _integrate(self, delta_time: float, now_wall: float) -> None:
        if now_wall - self._last_command > self._command_timeout:
            linear = 0.0
            angular = 0.0
        else:
            linear = self._linear_command
            angular = self._angular_command

        next_yaw = normalize_angle(self._odom_yaw + angular * delta_time)
        middle_yaw = self._odom_yaw + angular * delta_time / 2.0
        next_x = self._odom_x + linear * math.cos(middle_yaw) * delta_time
        next_y = self._odom_y + linear * math.sin(middle_yaw) * delta_time
        map_x, map_y, _ = self._map_pose(next_x, next_y, next_yaw)

        if self._pose_is_free(map_x, map_y):
            self._odom_x = next_x
            self._odom_y = next_y
            actual_linear = linear
        else:
            actual_linear = 0.0
            if now_wall - self._last_collision_log > 2.0:
                self.get_logger().warning('底盘接近占用栅格，已阻止向前运动')
                self._last_collision_log = now_wall

        self._odom_yaw = next_yaw
        self._actual_linear = actual_linear
        self._actual_angular = angular

        left_speed = (
            actual_linear - angular * self._wheel_separation / 2.0
        ) / self._wheel_radius
        right_speed = (
            actual_linear + angular * self._wheel_separation / 2.0
        ) / self._wheel_radius
        self._left_wheel_position += left_speed * delta_time
        self._right_wheel_position += right_speed * delta_time

    def _update(self) -> None:
        now_wall = time.monotonic()
        delta_time = min(0.1, max(0.0, now_wall - self._last_update))
        self._last_update = now_wall
        simulation_time = now_wall - self._wall_start
        stamp = self._stamp(simulation_time)

        clock = Clock()
        clock.clock = stamp
        self._clock_publisher.publish(clock)

        self._integrate(delta_time, now_wall)
        self._publish_motion(stamp)

        if simulation_time - self._last_scan >= 1.0 / self._scan_rate:
            self._publish_scan(stamp)
            self._last_scan = simulation_time

    def _publish_motion(self, stamp: TimeMessage) -> None:
        orientation_z, orientation_w = quaternion_from_yaw(self._odom_yaw)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = self._odom_x
        transform.transform.translation.y = self._odom_y
        transform.transform.rotation.z = orientation_z
        transform.transform.rotation.w = orientation_w
        self._tf_broadcaster.sendTransform(transform)

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = 'odom'
        odometry.child_frame_id = 'base_footprint'
        odometry.pose.pose.position.x = self._odom_x
        odometry.pose.pose.position.y = self._odom_y
        odometry.pose.pose.orientation.z = orientation_z
        odometry.pose.pose.orientation.w = orientation_w
        odometry.pose.covariance[0] = 0.01
        odometry.pose.covariance[7] = 0.01
        odometry.pose.covariance[35] = 0.02
        odometry.twist.twist.linear.x = self._actual_linear
        odometry.twist.twist.angular.z = self._actual_angular
        odometry.twist.covariance[0] = 0.01
        odometry.twist.covariance[35] = 0.02
        self._odom_publisher.publish(odometry)

        joints = JointState()
        joints.header.stamp = stamp
        joints.name = ['left_wheel_joint', 'right_wheel_joint']
        joints.position = [self._left_wheel_position, self._right_wheel_position]
        self._joint_publisher.publish(joints)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'imu_link'
        imu.orientation.z = orientation_z
        imu.orientation.w = orientation_w
        imu.orientation_covariance[0] = 0.02
        imu.orientation_covariance[4] = 0.02
        imu.orientation_covariance[8] = 0.04
        imu.angular_velocity.z = self._actual_angular
        imu.angular_velocity_covariance[8] = 0.02
        self._imu_publisher.publish(imu)

    def _publish_scan(self, stamp: TimeMessage) -> None:
        map_x, map_y, map_yaw = self._map_pose()
        laser_x = map_x + self._laser_offset_x * math.cos(map_yaw)
        laser_y = map_y + self._laser_offset_x * math.sin(map_yaw)
        ranges = []

        for relative_angle in self._scan_angles:
            ray_angle = map_yaw + relative_angle
            cosine = math.cos(ray_angle)
            sine = math.sin(ray_angle)
            distance = self._scan_min
            while distance < self._scan_max:
                ray_x = laser_x + distance * cosine
                ray_y = laser_y + distance * sine
                if self._map.is_occupied(ray_x, ray_y):
                    break
                distance += self._ray_step
            ranges.append(min(distance, self._scan_max))

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'laser_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = self._angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self._scan_rate
        scan.range_min = self._scan_min
        scan.range_max = self._scan_max
        scan.ranges = ranges
        scan.intensities = [1.0] * self._scan_samples
        self._scan_publisher.publish(scan)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = LightweightSimulator()
        rclpy.spin(node)
    except (FileNotFoundError, KeyError, ValueError) as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f'lightweight_simulator: {error}')
        raise
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

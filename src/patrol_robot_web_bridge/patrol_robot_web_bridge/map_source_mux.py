"""Publish exactly one selected OccupancyGrid on the canonical /map topic."""

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


class MapSourceMux(Node):
    """Select between live SLAM and the static Nav2 map server.

    Both upstream publishers stay available on private topics. Nav2, the web
    bridge and RViz continue to consume one stable /map topic, so switching
    modes never creates two competing OccupancyGrid publishers.
    """

    def __init__(self):
        super().__init__('map_source_mux')
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._selected = 'slam'
        self._latest = {'slam': None, 'static': None}
        self._publisher = self.create_publisher(
            OccupancyGrid,
            '/map',
            map_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            '/slam_map',
            lambda message: self._on_map('slam', message),
            map_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            '/static_map',
            lambda message: self._on_map('static', message),
            map_qos,
        )
        self.create_subscription(
            String,
            '/patrol/map_source/select',
            self._on_select,
            command_qos,
        )

    def _on_map(self, source, message):
        self._latest[source] = message
        if source == self._selected:
            self._publisher.publish(message)

    def _on_select(self, message):
        requested = str(message.data).strip().lower()
        if requested == 'clear_slam':
            self._latest['slam'] = None
            self.get_logger().info('已清空旧 SLAM 地图缓存，等待新会话首帧')
            return
        reset_slam_cache = requested == 'slam_reset'
        source = 'slam' if reset_slam_cache else requested
        if source not in self._latest:
            self.get_logger().error(f'不支持的地图源：{message.data}')
            return
        if reset_slam_cache:
            # The previous SLAM map belongs to the old scene. Wait for the
            # reset SLAM Toolbox instance to publish its first fresh map.
            self._latest['slam'] = None
        if source == self._selected:
            # The bridge periodically republishes the selected source so a
            # restarted mux can recover. A same-source heartbeat is not a
            # transition and must not flood logs or republish the map.
            return
        self._selected = source
        cached = self._latest[source]
        if cached is not None:
            self._publisher.publish(cached)
        self.get_logger().warning(
            '地图源已切换为：'
            + ('实时 SLAM' if source == 'slam' else 'Nav2 静态地图')
        )


def main(args=None):
    rclpy.init(args=args)
    node = MapSourceMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Synchronise web map scene objects into Gazebo Sim.

The web bridge publishes a complete scenario on /patrol/map_scenario.  This
node replaces only entities owned by the scenario editor and gives every
object matching visual and collision geometry.
"""

import json
import math
import queue
import re
import subprocess
import tempfile
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


OWNED_PREFIX = 'patrol_scene_'
LEGACY_MODELS = ('pipe_rack_a', 'pipe_rack_b', 'control_cabinet')


def bounded_number(value, fallback, minimum, maximum):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return fallback
    if not math.isfinite(number):
        return fallback
    return max(minimum, min(number, maximum))


def safe_fragment(value, fallback):
    fragment = re.sub(r'[^A-Za-z0-9_]+', '_', str(value)).strip('_')
    return (fragment[:48] or fallback)


class GazeboSceneSync(Node):
    def __init__(self):
        super().__init__('gazebo_scene_sync')
        self.declare_parameter('world_name', 'pipeline_inspection')
        self.declare_parameter('scenario_topic', '/patrol/map_scenario')
        self.declare_parameter('service_timeout_ms', 5000)
        self._world = safe_fragment(
            self.get_parameter('world_name').value,
            'pipeline_inspection',
        )
        self._timeout = max(
            1000,
            min(int(self.get_parameter('service_timeout_ms').value), 30000),
        )
        self._owned_names = set()
        self._generation = 0
        self._legacy_cleared = False
        self._updates = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self.create_subscription(
            String,
            str(self.get_parameter('scenario_topic').value),
            self._on_scenario,
            10,
        )
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f'Gazebo 场景同步已就绪：world={self._world}'
        )

    def destroy_node(self):
        self._stop.set()
        self._worker.join(timeout=1.0)
        return super().destroy_node()

    def _on_scenario(self, message):
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError('场景根节点必须是对象')
            objects = payload.get('objects', [])
            if not isinstance(objects, list) or len(objects) > 500:
                raise ValueError('场景对象格式无效或数量超过 500')
        except (json.JSONDecodeError, ValueError) as error:
            self.get_logger().error(f'拒绝无效 Gazebo 场景：{error}')
            return

        # A rapid sequence of editor updates only needs the newest full scene.
        try:
            self._updates.put_nowait(payload)
        except queue.Full:
            try:
                self._updates.get_nowait()
            except queue.Empty:
                pass
            self._updates.put_nowait(payload)

    def _run(self):
        while not self._stop.is_set():
            try:
                payload = self._updates.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self._apply_scene(payload)
            except (OSError, RuntimeError, ValueError) as error:
                self.get_logger().error(f'Gazebo 场景同步失败：{error}')

    def _gz_service(self, service, request_type, request):
        command = [
            'gz', 'service', '-s', service,
            '--reqtype', request_type,
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', str(self._timeout),
            '--req', request,
        ]
        completed = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=self._timeout / 1000.0 + 2.0,
        )
        output = f'{completed.stdout}\n{completed.stderr}'.lower()
        return completed.returncode == 0 and 'data: true' in output

    def _remove(self, name, required=False):
        request = f'name: "{name}", type: MODEL'
        removed = self._gz_service(
            f'/world/{self._world}/remove',
            'gz.msgs.Entity',
            request,
        )
        if required and not removed:
            raise RuntimeError(f'无法删除旧场景实体 {name}')

    @staticmethod
    def _sdf(name, item):
        width = bounded_number(item.get('width'), 1.0, 0.10, 50.0)
        depth = bounded_number(item.get('depth'), 1.0, 0.10, 50.0)
        height = bounded_number(item.get('height'), 1.0, 0.10, 20.0)
        x = bounded_number(item.get('x'), 0.0, -10000.0, 10000.0)
        y = bounded_number(item.get('y'), 0.0, -10000.0, 10000.0)
        if item.get('type') == 'device':
            ambient, diffuse = '0.10 0.34 0.55 1', '0.14 0.48 0.72 1'
        else:
            ambient, diffuse = '0.58 0.20 0.08 1', '0.88 0.34 0.10 1'
        return f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <pose>{x:.6f} {y:.6f} {height / 2.0:.6f} 0 0 0</pose>
    <link name="body">
      <collision name="collision">
        <geometry><box><size>{width:.6f} {depth:.6f} {height:.6f}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{width:.6f} {depth:.6f} {height:.6f}</size></box></geometry>
        <material><ambient>{ambient}</ambient><diffuse>{diffuse}</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
'''

    def _spawn(self, name, item):
        with tempfile.TemporaryDirectory(prefix='patrol_scene_') as directory:
            path = Path(directory) / f'{name}.sdf'
            path.write_text(self._sdf(name, item), encoding='utf-8')
            request = f'sdf_filename: "{path}", name: "{name}"'
            created = self._gz_service(
                f'/world/{self._world}/create',
                'gz.msgs.EntityFactory',
                request,
            )
        if not created:
            raise RuntimeError(f'无法创建场景实体 {name}')

    def _apply_scene(self, payload):
        if not self._legacy_cleared:
            for name in LEGACY_MODELS:
                self._remove(name)
            self._legacy_cleared = True

        for name in tuple(self._owned_names):
            self._remove(name, required=True)
        self._owned_names.clear()

        objects = payload.get('objects', [])
        created = []
        self._generation += 1
        try:
            for index, item in enumerate(objects):
                if not isinstance(item, dict):
                    continue
                fragment = safe_fragment(item.get('id'), f'object_{index + 1}')
                # A new generation avoids a create/delete race because Gazebo
                # applies entity removal at the end of a simulation step.
                name = (
                    f'{OWNED_PREFIX}g{self._generation}_'
                    f'{index + 1}_{fragment}'
                )
                self._spawn(name, item)
                created.append(name)
        except (OSError, RuntimeError, ValueError):
            for name in created:
                self._remove(name)
            raise

        self._owned_names.update(created)
        self.get_logger().warning(
            f'Gazebo 场景已切换：{payload.get("name", "未命名地图")}，'
            f'已创建 {len(created)} 个带碰撞实体'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GazeboSceneSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

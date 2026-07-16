import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";

async function render() {
  const workerUrl = new URL("../dist/server/index.js", import.meta.url);
  workerUrl.searchParams.set("test", `${process.pid}-${Date.now()}`);
  const { default: worker } = await import(workerUrl.href);

  return worker.fetch(
    new Request("http://localhost/", {
      headers: { accept: "text/html" },
    }),
    {
      ASSETS: {
        fetch: async () => new Response("Not found", { status: 404 }),
      },
    },
    {
      waitUntil() {},
      passThroughOnException() {},
    },
  );
}

test("server-renders the patrol robot control console", async () => {
  const response = await render();
  assert.equal(response.status, 200);
  assert.match(response.headers.get("content-type") ?? "", /^text\/html\b/i);

  const html = await response.text();
  assert.match(html, /<title>巡检智控 · 车辆控制台<\/title>/i);
  assert.match(html, /车辆控制台/);
  assert.match(html, /2D 地图/);
  assert.match(html, /3D 场景/);
  assert.match(html, /摄像画面/);
  assert.match(html, /导航感知模式/);
  assert.match(html, /雷达模式/);
  assert.match(html, /视觉模式/);
  assert.match(html, /融合模式/);
  assert.match(html, /人工控制/);
  assert.match(html, /前进/);
  assert.match(html, /后退/);
  assert.match(html, /地图管理/);
});

test("includes live ROS controls and both map implementations", async () => {
  const [page, map2d, map3d, mapManagement, mapTypes, bridge, simulator, nav2, gazeboSync, gazeboLaunch, bringup, patrolManager] = await Promise.all([
    readFile(new URL("../app/page.tsx", import.meta.url), "utf8"),
    readFile(new URL("../app/Industrial2DMap.tsx", import.meta.url), "utf8"),
    readFile(new URL("../app/Industrial3DMap.tsx", import.meta.url), "utf8"),
    readFile(new URL("../app/MapManagement.tsx", import.meta.url), "utf8"),
    readFile(new URL("../app/mapTypes.ts", import.meta.url), "utf8"),
    readFile(new URL("../../patrol_robot_web_bridge/patrol_robot_web_bridge/bridge_node.py", import.meta.url), "utf8"),
    readFile(new URL("../../patrol_robot_simulator/patrol_robot_simulator/simulator_node.py", import.meta.url), "utf8"),
    readFile(new URL("../../patrol_robot_navigation/config/nav2_params.yaml", import.meta.url), "utf8"),
    readFile(new URL("../../patrol_robot_gazebo/scripts/gazebo_scene_sync.py", import.meta.url), "utf8"),
    readFile(new URL("../../patrol_robot_gazebo/launch/simulation.launch.py", import.meta.url), "utf8"),
    readFile(new URL("../../patrol_robot_bringup/launch/simulation_navigation.launch.py", import.meta.url), "utf8"),
    readFile(new URL("../../patrol_robot_patrol/patrol_robot_patrol/patrol_manager.py", import.meta.url), "utf8"),
  ]);

  assert.match(page, /\/api\/status/);
  assert.match(page, /\/api\/control\/manual/);
  assert.match(page, /\/api\/navigation\/waypoints/);
  assert.match(page, /\/api\/camera\/enable/);
  assert.match(page, /\/api\/camera\/stream/);
  assert.match(page, /\/api\/camera\/gimbal/);
  assert.match(page, /\/api\/perception\/mode/);
  assert.match(page, /\/api\/maps\/activate/);
  assert.match(page, /开启摄像头/);
  assert.match(page, /回到正前方/);
  assert.match(page, /<Industrial2DMap/);
  assert.match(page, /<Industrial3DMap/);
  assert.match(map2d, /industrial-2d-map/);
  assert.match(map2d, /巡检工程平面图/);
  assert.match(map3d, /industrial-3d-map/);
  assert.match(map3d, /onMoveEntity/);
  assert.match(map3d, /dragStateRef/);
  assert.match(mapManagement, /导入地图/);
  assert.match(mapManagement, /随机地图种子/);
  assert.match(mapManagement, /障碍物/);
  assert.match(mapManagement, /设备/);
  assert.match(mapManagement, /巡检点/);
  assert.match(mapManagement, /位置微调/);
  assert.match(mapManagement, /向左移动/);
  assert.match(mapTypes, /generatePatrolMap/);
  assert.match(mapTypes, /parsePgm/);
  assert.match(mapTypes, /validatePatrolWaypoints/);
  assert.match(mapTypes, /WAYPOINT_SAFETY_RADIUS = 0\.45/);
  assert.match(bridge, /\/api\/control\/manual/);
  assert.match(bridge, /\/api\/camera\/enable/);
  assert.match(bridge, /\/api\/camera\/stream/);
  assert.match(bridge, /\/api\/camera\/gimbal/);
  assert.match(bridge, /\/api\/perception\/mode/);
  assert.match(bridge, /\/api\/maps\/activate/);
  assert.match(bridge, /LoadMap/);
  assert.match(bridge, /\/patrol\/map_scenario/);
  assert.match(bridge, /PoseWithCovarianceStamped/);
  assert.match(bridge, /\/initialpose/);
  assert.match(bridge, /odom_pose_is_world/);
  assert.match(bridge, /TransformBroadcaster/);
  assert.match(bridge, /ground_truth_localization/);
  assert.match(bridge, /lidar_obstacle_layer\.enabled/);
  assert.match(bridge, /camera_voxel_layer\.enabled/);
  assert.match(bridge, /camera_stream_fps', 12\.0/);
  assert.match(bridge, /manual_command_timeout/);
  assert.match(simulator, /OccupancyMap\.from_scenario/);
  assert.match(simulator, /\/patrol\/map_scenario/);
  assert.match(gazeboSync, /OWNED_PREFIX = 'patrol_scene_'/);
  assert.match(gazeboSync, /gz\.msgs\.EntityFactory/);
  assert.match(gazeboSync, /request = f'sdf: \{json\.dumps\(sdf\)\}/);
  assert.doesNotMatch(gazeboSync, /TemporaryDirectory/);
  assert.match(gazeboSync, /<collision name="collision">/);
  assert.match(gazeboLaunch, /executable='gazebo_scene_sync'/);
  assert.match(bringup, /DeclareLaunchArgument\('use_gazebo', default_value='true'\)/);
  assert.match(patrolManager, /loop_count/);
  assert.match(patrolManager, /returning_home/);
  assert.match(patrolManager, /NavigateThroughPoses/);
  assert.match(patrolManager, /开始连续路线规划/);
  assert.match(page, /巡检圈数/);
  assert.match(page, /地图定位尚未稳定/);
  assert.match(nav2, /plugins: \[lidar_obstacle_layer, camera_voxel_layer, inflation_layer\]/);
  assert.match(nav2, /plugin: nav2_costmap_2d::VoxelLayer/);
  assert.match(nav2, /topic: \/camera\/points\/filtered/);
  assert.match(nav2, /tf_broadcast: false/);
  assert.match(nav2, /nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController/);
  assert.match(nav2, /nav2_smac_planner::SmacPlanner2D/);
  assert.match(bringup, /ground_truth_localization/);
});

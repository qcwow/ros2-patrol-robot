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
  assert.match(html, /自主建图/);
});

test("includes live ROS controls and both map implementations", async () => {
  const [page, map2d, map3d, autonomousMapping, liveMapping, mapManagement, mapTypes, bridge, mapSourceMux, simulator, nav2, navLaunch, gazeboSync, gazeboLaunch, robotDescription, bringup, mappingBringup, patrolManager, navigationHealth, footprintGeometry, taskLedger, patrolTree] = await Promise.all([
    readFile(new URL("../app/page.tsx", import.meta.url), "utf8"),
    readFile(new URL("../app/Industrial2DMap.tsx", import.meta.url), "utf8"),
    readFile(new URL("../app/Industrial3DMap.tsx", import.meta.url), "utf8"),
    readFile(new URL("../app/AutonomousMappingWorkspace.tsx", import.meta.url), "utf8"),
    readFile(new URL("../app/LiveMappingWorkspace.tsx", import.meta.url), "utf8"),
    readFile(new URL("../app/MapManagement.tsx", import.meta.url), "utf8"),
    readFile(new URL("../app/mapTypes.ts", import.meta.url), "utf8"),
    readFile(new URL("../../patrol_robot_web_bridge/patrol_robot_web_bridge/bridge_node.py", import.meta.url), "utf8"),
    readFile(new URL("../../patrol_robot_web_bridge/patrol_robot_web_bridge/map_source_mux.py", import.meta.url), "utf8"),
    readFile(new URL("../../patrol_robot_simulator/patrol_robot_simulator/simulator_node.py", import.meta.url), "utf8"),
    readFile(new URL("../../patrol_robot_navigation/config/nav2_params.yaml", import.meta.url), "utf8"),
    readFile(new URL("../../patrol_robot_navigation/launch/navigation.launch.py", import.meta.url), "utf8"),
    readFile(new URL("../../patrol_robot_gazebo/scripts/gazebo_scene_sync.py", import.meta.url), "utf8"),
    readFile(new URL("../../patrol_robot_gazebo/launch/simulation.launch.py", import.meta.url), "utf8"),
    readFile(new URL("../../patrol_robot_description/urdf/patrol_robot.urdf.xacro", import.meta.url), "utf8"),
    readFile(new URL("../../patrol_robot_bringup/launch/simulation_navigation.launch.py", import.meta.url), "utf8"),
    readFile(new URL("../../patrol_robot_bringup/launch/simulation_mapping.launch.py", import.meta.url), "utf8"),
    readFile(new URL("../../patrol_robot_patrol/patrol_robot_patrol/patrol_manager.py", import.meta.url), "utf8"),
    readFile(new URL("../../patrol_robot_patrol/patrol_robot_patrol/navigation_health_monitor.py", import.meta.url), "utf8"),
    readFile(new URL("../../patrol_robot_patrol/patrol_robot_patrol/footprint_geometry.py", import.meta.url), "utf8"),
    readFile(new URL("../../patrol_robot_patrol/patrol_robot_patrol/task_ledger.py", import.meta.url), "utf8"),
    readFile(new URL("../../patrol_robot_patrol/behavior_trees/navigate_to_pose_stable.xml", import.meta.url), "utf8"),
  ]);

  assert.match(page, /\/api\/status/);
  assert.match(page, /\/api\/control\/manual/);
  assert.match(page, /\/api\/navigation\/waypoints/);
  assert.match(page, /\/api\/camera\/enable/);
  assert.match(page, /\/api\/camera\/stream/);
  assert.match(page, /\/api\/camera\/gimbal/);
  assert.match(page, /\/api\/perception\/mode/);
  assert.match(page, /\/api\/maps\/activate/);
  assert.match(page, /\/api\/maps\/active/);
  assert.match(page, /hasPendingMapChanges/);
  assert.match(page, /车辆当前地图/);
  assert.match(page, /APPLIED_MAP_STORAGE_KEY/);
  assert.match(page, /max_linear_speed/);
  assert.match(page, /speedGaugePercent/);
  assert.match(page, /speed-gauge-progress/);
  assert.match(page, /strokeDasharray/);
  assert.match(page, /开启摄像头/);
  assert.match(page, /回到正前方/);
  assert.match(page, /<Industrial2DMap/);
  assert.match(page, /<Industrial3DMap/);
  assert.match(map2d, /industrial-2d-map/);
  assert.match(map2d, /巡检工程平面图/);
  assert.match(map2d, /navigationPath/);
  assert.match(map2d, /Nav2 当前实际规划路径/);
  assert.match(map3d, /industrial-3d-map/);
  assert.match(map3d, /onMoveEntity/);
  assert.match(map3d, /dragStateRef/);
  assert.match(autonomousMapping, /OrbitControls/);
  assert.match(autonomousMapping, /InstancedMesh/);
  assert.match(autonomousMapping, /开始自主探索建图/);
  assert.match(autonomousMapping, /结束并保存地图/);
  assert.match(autonomousMapping, /放弃当前建图/);
  assert.match(autonomousMapping, /显示机器人视场角 FOV/);
  assert.match(autonomousMapping, /显示激光雷达射线/);
  assert.match(autonomousMapping, /\/api\/mapping\/start/);
  assert.match(autonomousMapping, /\/api\/mapping\/finish/);
  assert.match(autonomousMapping, /\/api\/mapping\/discard/);
  assert.match(autonomousMapping, /应用此地图 \/ 部署/);
  assert.match(liveMapping, /\/api\/mapping\/map/);
  assert.match(liveMapping, /\/api\/mapping\/explore/);
  assert.match(liveMapping, /\/api\/mapping\/finish/);
  assert.match(liveMapping, /\/api\/mapping\/deploy/);
  assert.match(liveMapping, /\/api\/control\/manual/);
  assert.match(liveMapping, /decodeRuns/);
  assert.match(liveMapping, /实时二维 SLAM 栅格地图/);
  assert.match(liveMapping, /相机实时画面/);
  assert.match(liveMapping, /人工方向控制/);
  assert.match(liveMapping, /按住方向键移动/);
  assert.match(liveMapping, /mapping-drive-forward/);
  assert.match(liveMapping, /mapping-drive-left/);
  assert.match(liveMapping, /mapping-drive-stop/);
  assert.match(liveMapping, /mapping-drive-right/);
  assert.match(liveMapping, /mapping-drive-reverse/);
  assert.match(liveMapping, /一键自主探路/);
  assert.match(liveMapping, /blockedByOtherTask/);
  assert.match(liveMapping, /巡检任务运行中/);
  assert.match(liveMapping, /应用此地图 \/ 部署/);
  assert.match(liveMapping, /设置巡检点/);
  assert.match(liveMapping, /2D 栅格 \+ 3D OctoMap/);
  assert.match(liveMapping, /车辆网关未连接/);
  assert.match(liveMapping, /连接车辆/);
  assert.match(page, /changeRobotGateway/);
  assert.match(page, /telemetry\.operation\.locked/);
  assert.match(page, /建图任务运行中/);
  assert.match(page, /恢复应用地图并巡检/);
  assert.match(mapManagement, /导入地图/);
  assert.match(mapManagement, /onSelect/);
  assert.match(mapManagement, /onApply/);
  assert.match(mapManagement, /应用巡检路线/);
  assert.match(mapManagement, /地图与路线已应用/);
  assert.match(mapManagement, /实时 SLAM 地图已在车辆内存中/);
  assert.match(mapManagement, /map-card-delete/);
  assert.match(mapManagement, /不安全巡检点/);
  assert.match(mapManagement, /随机地图种子/);
  assert.match(mapManagement, /障碍物/);
  assert.match(mapManagement, /设备/);
  assert.match(mapManagement, /巡检点/);
  assert.match(mapManagement, /新建 SLAM 地图已同步/);
  assert.match(mapManagement, /过渡点/);
  assert.match(mapManagement, /TRANSIT/);
  assert.match(mapManagement, /count_as_task: false/);
  assert.match(mapManagement, /位置微调/);
  assert.match(mapManagement, /向左移动/);
  assert.match(mapManagement, /map-safety-alert/);
  assert.match(mapManagement, /不安全巡检点/);
  assert.match(mapManagement, /validatePatrolWaypoints/);
  assert.match(mapTypes, /generatePatrolMap/);
  assert.match(mapTypes, /parsePgm/);
  assert.match(mapTypes, /validatePatrolWaypoints/);
  assert.match(mapTypes, /WAYPOINT_SAFETY_RADIUS = 0\.45/);
  assert.match(mapTypes, /GAZEBO_BOUNDARY_WALL_INSET = 0\.50/);
  assert.match(mappingBringup, /executable='patrol_manager'/);
  assert.match(mappingBringup, /executable='navigation_health_monitor'/);
  assert.match(mappingBringup, /executable='map_source_mux'/);
  assert.match(mappingBringup, /executable='map_server'/);
  assert.match(mappingBringup, /executable='amcl'/);
  assert.match(mappingBringup, /'tf_broadcast': False/);
  assert.match(mappingBringup, /'patrol_route_ready_at_start': False/);
  assert.match(mappingBringup, /'autonomous_exploration_available'/);
  assert.match(mappingBringup, /'scan_samples': '360'/);
  assert.match(mapSourceMux, /\/slam_map/);
  assert.match(mapSourceMux, /\/static_map/);
  assert.match(mapSourceMux, /\/patrol\/map_source\/select/);
  assert.match(mapSourceMux, /clear_slam/);
  assert.match(bridge, /patrol_route_ready/);
  assert.match(bridge, /MAPPING_OPERATION_STATES/);
  assert.match(bridge, /拒绝同时启动巡检/);
  assert.match(bridge, /拒绝同时启动建图/);
  assert.match(bridge, /_mapping_base_payload/);
  assert.match(bridge, /_pending_patrol_start/);
  assert.match(bridge, /_scenario_route_connectivity_issues/);
  assert.match(bridge, /cv2\.distanceTransform/);
  assert.match(bridge, /_prepare_slam_mapping_mode/);
  assert.match(bridge, /TRANSITION_DEACTIVATE/);
  assert.match(bridge, /TRANSITION_ACTIVATE/);
  assert.match(mapTypes, /WAYPOINT_BOUNDARY_CLEARANCE/);
  assert.match(mapTypes, /withRouteHeadings/);
  assert.match(mapTypes, /withWaypointSemantics/);
  assert.match(page, /withRouteHeadings\(next\)/);
  assert.match(bridge, /\/api\/control\/manual/);
  assert.match(bridge, /\/api\/camera\/enable/);
  assert.match(bridge, /\/api\/camera\/stream/);
  assert.match(bridge, /\/api\/camera\/gimbal/);
  assert.match(bridge, /\/api\/perception\/mode/);
  assert.match(bridge, /\/api\/maps\/activate/);
  assert.match(bridge, /\/api\/maps\/active/);
  assert.match(bridge, /\/api\/mapping\/map/);
  assert.match(bridge, /\/api\/mapping\/maps/);
  assert.match(bridge, /path\.startswith\('\/api\/mapping\/'\)/);
  assert.match(bridge, /_save_mapping_map/);
  assert.match(bridge, /_deploy_mapping_map/);
  assert.match(bridge, /OccupancyGrid/);
  assert.match(bridge, /rle-int8/);
  assert.match(bridge, /SaveMap/);
  assert.match(bridge, /_pending_map_payload/);
  assert.match(bridge, /LoadMap/);
  assert.match(bridge, /\/patrol\/map_scenario/);
  assert.match(bridge, /_scenario_waypoint_issues/);
  assert.match(bridge, /_with_route_headings/);
  assert.match(bridge, /WAYPOINT_BOUNDARY_CLEARANCE/);
  assert.match(bridge, /perimeter_cells/);
  assert.match(bridge, /PoseWithCovarianceStamped/);
  assert.match(bridge, /\/initialpose/);
  assert.match(bridge, /odom_pose_is_world/);
  assert.match(bridge, /TransformBroadcaster/);
  assert.match(bridge, /ground_truth_localization/);
  assert.match(bridge, /\/ground_truth\/odom/);
  assert.match(bridge, /used only to seed AMCL after a simulated map change/);
  assert.match(bridge, /pending_route is not None and not scene_ready/);
  assert.match(bridge, /map_x = float\(route_home\['x'\]\)/);
  assert.match(bridge, /lidar_obstacle_layer\.enabled/);
  assert.match(bridge, /camera_obstacle_layer\.enabled/);
  assert.match(bridge, /camera_stream_fps', 12\.0/);
  assert.match(bridge, /Path as NavPath/);
  assert.match(bridge, /\/plan_smoothed/);
  assert.match(bridge, /_on_raw_navigation_path/);
  assert.match(bridge, /_store_navigation_path/);
  assert.match(bridge, /_on_navigation_path/);
  assert.match(bridge, /_resample_occupancy/);
  assert.match(bridge, /NAVIGATION_GRID_TARGET_RESOLUTION = 0\.05/);
  assert.match(bridge, /camera_cloud_timeout_seconds/);
  assert.match(bridge, /perception_fault_delay_seconds/);
  assert.match(bridge, /perception_recovery_stable_seconds/);
  assert.match(bridge, /_release_perception_hold_if_ready/);
  assert.match(bridge, /感知恢复，正在自动恢复原巡检任务/);
  assert.match(bridge, /Do not silently lose Start\/Stop/);
  assert.match(bridge, /manual_command_timeout/);
  assert.match(bridge, /\/cmd_vel_nav/);
  assert.match(bridge, /_on_navigation_command/);
  assert.match(bridge, /_publish_base_command/);
  assert.match(bridge, /output\.angular\.z = command\.angular\.z/);
  assert.match(bridge, /successful perception transition releases this latch/);
  assert.doesNotMatch(page, /className="emergency"/);
  assert.match(page, /当前视野没有障碍不会停车/);
  assert.match(simulator, /OccupancyMap\.from_scenario/);
  assert.match(simulator, /\/patrol\/map_scenario/);
  assert.match(simulator, /\/patrol\/map_scenario_status/);
  assert.match(simulator, /_publish_scenario_status/);
  assert.match(simulator, /_pose_is_free_on_map/);
  assert.match(gazeboSync, /OWNED_PREFIX = 'patrol_scene_'/);
  assert.match(gazeboSync, /gz\.msgs\.EntityFactory/);
  assert.match(gazeboSync, /gz\.msgs\.Pose/);
  assert.match(gazeboSync, /\/set_pose/);
  assert.match(gazeboSync, /_teleport_robot_home/);
  assert.match(gazeboSync, /Aim at the next distinct task/);
  assert.match(gazeboSync, /request = f'sdf: \{json\.dumps\(sdf\)\}/);
  assert.doesNotMatch(gazeboSync, /TemporaryDirectory/);
  assert.match(gazeboSync, /<collision name="collision">/);
  assert.match(gazeboLaunch, /executable='gazebo_scene_sync'/);
  assert.match(robotDescription, /<left_joint>left_wheel_joint<\/left_joint>/);
  assert.match(robotDescription, /<right_joint>right_wheel_joint<\/right_joint>/);
  assert.match(bringup, /DeclareLaunchArgument\('use_gazebo', default_value='true'\)/);
  assert.match(bringup, /'perception_initial_mode': 'lidar'/);
  assert.match(patrolManager, /loop_count/);
  assert.match(patrolManager, /returning_home/);
  assert.match(patrolManager, /NavigateToPose/);
  assert.match(patrolManager, /开始单点导航/);
  assert.match(patrolManager, /goal\.pose = pose/);
  assert.match(patrolManager, /waypoint_tasks/);
  assert.match(patrolManager, /PatrolTaskLedger/);
  assert.match(patrolManager, /BLOCKED/);
  assert.match(patrolManager, /ESTOP/);
  assert.match(patrolManager, /ComputePathToPose/);
  assert.match(patrolManager, /WAITING_HEALTH/);
  assert.match(patrolManager, /WAITING_SENSOR/);
  assert.match(patrolManager, /automatic_retry/);
  assert.match(patrolManager, /0\.80 \*\* \(\s*self\._retry_count \+ self\._health_recovery_count/);
  assert.match(patrolManager, /failure_detail=f'路径预检失败/);
  assert.doesNotMatch(patrolManager, /retry_cooldown_seconds/);
  assert.match(patrolManager, /lap_restart_delay_seconds/);
  assert.match(patrolManager, /ClearEntireCostmap/);
  assert.match(patrolManager, /人工确认继续/);
  assert.match(patrolManager, /HEALTH_RECOVERY/);
  assert.match(patrolManager, /导航健康已持续稳定，正在自动低速重试当前路线点/);
  assert.match(patrolManager, /health_recovery_reset_progress_meters/);
  assert.match(patrolManager, /连续健康恢复记录/);
  assert.match(page, /重新检查并恢复/);
  assert.match(page, /安全停车检查中 · 健康稳定后自动续行/);
  assert.match(page, /低速安全验证中/);
  assert.match(page, /正在排除重复路线/);
  assert.match(bridge, /similar_path_replan_count/);
  assert.match(page, /health_recovery_reset_pending/);
  assert.match(page, /telemetry\.patrol\.state !== "BLOCKED"/);
  assert.match(navigationHealth, /footprint_length/);
  assert.match(navigationHealth, /footprint_collision_confirm_seconds/);
  assert.match(navigationHealth, /footprint_raw_clear/);
  assert.doesNotMatch(navigationHealth, />= 99/);
  assert.match(footprintGeometry, /lethal_cost_threshold: int = 100/);
  assert.match(footprintGeometry, /rectangle_overlaps_lethal_cell/);
  assert.match(taskLedger, /complete_inspection/);
  assert.match(taskLedger, /round_ready/);
  assert.match(patrolManager, /goal\.behavior_tree/);
  assert.match(patrolTree, /<ComputePathToPose/);
  assert.doesNotMatch(patrolTree, /<SmoothPath/);
  assert.doesNotMatch(patrolTree, /FollowStablePath/);
  assert.match(patrolTree, /shortest remaining route/i);
  assert.match(patrolTree, /GoalCheckerSelector/);
  assert.doesNotMatch(patrolTree, /RateController/);
  assert.match(page, /巡检圈数/);
  assert.match(page, /remaining_visits/);
  assert.match(page, /基地 · 出发 \/ 返回/);
  assert.match(page, /telemetry\.patrol\.loop_count !== patrolLoops/);
  assert.match(page, /地图定位尚未稳定/);
  assert.match(page, /telemetry\.navigation\.path/);
  assert.match(page, /转换为 0\.05 m\/格，但不会增加原图缺失的细节/);
  assert.match(nav2, /plugins: \[lidar_obstacle_layer, camera_obstacle_layer, inflation_layer\]/);
  assert.match(nav2, /plugins: \[static_layer, failed_path_layer, inflation_layer\]/);
  assert.match(nav2, /topic: \/patrol_manager\/failed_path_points/);
  assert.match(nav2, /sensor_frame: map/);
  assert.match(nav2, /error_code_names: \[compute_path_error_code, follow_path_error_code\]/);
  assert.match(patrolManager, /FailedPathMemory/);
  assert.match(patrolManager, /_handle_route_failure/);
  assert.match(patrolManager, /不对原路线执行低速重试/);
  assert.match(page, /当前路线已禁用 · 正在规划未使用的最短路线/);
  assert.doesNotMatch(nav2, /plugin: nav2_costmap_2d::VoxelLayer/);
  assert.match(nav2, /topic: \/camera\/points\/filtered/);
  assert.match(nav2, /tf_broadcast: false/);
  assert.match(nav2, /nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController/);
  assert.match(nav2, /nav2_rotation_shim_controller::RotationShimController/);
  assert.match(nav2, /angular_disengage_threshold: 0\.20/);
  assert.match(nav2, /closed_loop: false/);
  assert.match(nav2, /max_robot_pose_search_dist: 1\.5/);
  assert.match(nav2, /use_rotate_to_heading: true/);
  assert.match(nav2, /controller_frequency: 10\.0/);
  assert.match(nav2, /failure_tolerance: 1\.5/);
  assert.match(nav2, /max_allowed_time_to_collision_up_to_carrot: 0\.50/);
  assert.match(nav2, /use_cost_regulated_linear_velocity_scaling: true/);
  assert.match(nav2, /inflation_radius: 0\.40/);
  assert.match(nav2, /footprint_padding: 0\.01/);
  assert.match(nav2, /cost_scaling_factor: 5\.0/);
  assert.match(nav2, /feedback: OPEN_LOOP/);
  assert.match(nav2, /camera_obstacle_layer:[\s\S]*?enabled: false/);
  assert.match(navLaunch, /cmd_vel_nav_raw/);
  assert.match(navLaunch, /cmd_vel_smoothed', 'cmd_vel_nav/);
  assert.match(navLaunch, /'nav2_behaviors'[\s\S]*?\('cmd_vel', 'cmd_vel_nav_raw'\)/);
  assert.match(page, /startManual\(0, 0\.6, "左转"\)/);
  assert.match(page, /startManual\(0, -0\.6, "右转"\)/);
  assert.match(nav2, /nav2_navfn_planner::NavfnPlanner/);
  assert.match(nav2, /use_astar: false/);
  assert.match(nav2, /nav2_controller::PoseProgressChecker/);
  assert.match(bringup, /navigate_to_pose_stable\.xml/);
  assert.match(patrolTree, /NavigateWithStablePath/);
  assert.doesNotMatch(patrolTree, /number_of_retries="1"/);
  assert.doesNotMatch(patrolTree, /BackUp backup_dist="0\.25"/);
  assert.match(patrolTree, /Retrying here would follow it again/);
  assert.match(patrolManager, /_reject_similar_candidate/);
  assert.match(patrolManager, /last_candidate_similarity/);
  assert.match(patrolManager, /self\._active_plan = candidate/);
  assert.match(patrolManager, /self\._last_robot_position or self\._active_plan\[0\]/);
  assert.match(bringup, /navigate_to_pose_no_spin\.xml/);
  assert.match(bringup, /navigate_to_pose_restricted\.xml/);
  assert.match(bringup, /navigation_health_monitor/);
  assert.match(bringup, /'costmap_timeout_seconds': 8\.0/);
  assert.match(bringup, /'footprint_safety_margin': 0\.01/);
  assert.match(bringup, /navigation_regression_recorder/);
  assert.match(bringup, /default_value='ekf'/);
  assert.match(bringup, /package='robot_localization'/);
  assert.doesNotMatch(patrolTree, /RateController/);
  assert.match(bringup, /ground_truth_localization/);
});

"use client";

/* eslint-disable @next/next/no-img-element -- MJPEG streams require a native img element. */

import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { Industrial3DMap } from "./Industrial3DMap";
import { Industrial2DMap } from "./Industrial2DMap";
import { MapManagement } from "./MapManagement";
import { DEFAULT_MAPS, findNearestSafeWaypointPosition, mapToRobotPayload, validatePatrolWaypoints, type PatrolMap, type Waypoint } from "./mapTypes";

type CameraStatus = {
  enabled: boolean; ok: boolean; frames: number; width: number; height: number;
  last_frame_age: number | null; topic: string; error?: string | null; fps: number;
  stream_fps: number; pan_deg: number; tilt_deg: number; pan_target_deg: number;
  tilt_target_deg: number; gimbal_ok: boolean;
};
type PerceptionMode = "lidar" | "camera" | "fusion";
type PerceptionStatus = {
  mode: PerceptionMode; lidar_enabled: boolean; camera_enabled: boolean;
  lidar_ok: boolean; camera_ok: boolean; transitioning: boolean;
  gimbal_locked: boolean; safety_ok: boolean; camera_points: number;
  active_sources: string[]; last_camera_cloud_age: number | null;
  error?: string | null;
};
type MapRuntimeStatus = {
  active_id: string; active_name: string; transitioning: boolean;
  localization_ready?: boolean; error?: string | null;
};
type Telemetry = {
  speed: number; x: number; y: number; yaw: number; lidar_ok: boolean;
  patrol: {
    state: string; current_index: number; current_waypoint: string;
    waypoint_count: number; loop_count?: number; completed_loops?: number;
    returning_home?: boolean;
  };
  camera: CameraStatus;
  perception: PerceptionStatus;
  map: MapRuntimeStatus;
};

const CAMERA_LIMITS = { pan: 90, tiltUp: 25, tiltDown: 35 };
const PERCEPTION_MODES: Array<{ id: PerceptionMode; name: string; description: string; icon: string }> = [
  { id: "lidar", name: "雷达模式", description: "仅激光雷达参与避障", icon: "⌁" },
  { id: "camera", name: "视觉模式", description: "仅 RGB-D 点云参与避障", icon: "◉" },
  { id: "fusion", name: "融合模式", description: "雷达与视觉同时工作", icon: "◎" },
];

export default function Home() {
  const [speed, setSpeed] = useState(0.6);
  const [height, setHeight] = useState(16);
  const [length, setLength] = useState(52);
  const [width, setWidth] = useState(42);
  const [lidars, setLidars] = useState(1);
  const [running, setRunning] = useState(false);
  const [patrolLoops, setPatrolLoops] = useState(1);
  const [maps, setMaps] = useState<PatrolMap[]>(DEFAULT_MAPS);
  const [activeMapId, setActiveMapId] = useState(DEFAULT_MAPS[0].id);
  const [mapStorageReady, setMapStorageReady] = useState(false);
  const [activeSection, setActiveSection] = useState<"control" | "maps">("control");
  const activeMap = useMemo(() => maps.find((map) => map.id === activeMapId) ?? maps[0] ?? DEFAULT_MAPS[0], [activeMapId, maps]);
  const waypoints = activeMap.waypoints;
  const [selected, setSelected] = useState(3);
  const [toast, setToast] = useState("所有系统运行正常");
  const [connected, setConnected] = useState(false);
  const [telemetry, setTelemetry] = useState<Telemetry>({
    speed: 0, x: -6, y: -4, yaw: 0, lidar_ok: false,
    patrol: { state: "UNKNOWN", current_index: 0, current_waypoint: "等待连接", waypoint_count: 0 },
    camera: {
      enabled: false, ok: false, frames: 0, width: 0, height: 0, fps: 0,
      stream_fps: 12, last_frame_age: null, topic: "/camera/color/image_raw",
      pan_deg: 0, tilt_deg: 0, pan_target_deg: 0, tilt_target_deg: 0, gimbal_ok: false,
    },
    perception: {
      mode: "fusion", lidar_enabled: true, camera_enabled: true,
      lidar_ok: false, camera_ok: false, transitioning: false,
      gimbal_locked: false, safety_ok: true, camera_points: 0,
      active_sources: [], last_camera_cloud_age: null,
    },
    map: { active_id: "pipeline-demo", active_name: "管廊综合测试区", transitioning: false, localization_ready: true, error: null },
  });
  const [apiBase] = useState(() => {
    if (typeof window === "undefined") return "";
    const query = new URLSearchParams(window.location.search).get("robot");
    const stored = window.localStorage.getItem("robot_api_url");
    const fallback = `http://${window.location.hostname}:8765`;
    const base = (query || stored || fallback).replace(/\/$/, "");
    if (query) window.localStorage.setItem("robot_api_url", base);
    return base;
  });
  const [now, setNow] = useState<Date | null>(null);
  const [mapZoom, setMapZoom] = useState(1);
  const [mapMode, setMapMode] = useState<"2d" | "3d" | "camera">("2d");
  const [manualActive, setManualActive] = useState("未接管");
  const [cameraEnabled, setCameraEnabled] = useState(false);
  const [cameraReady, setCameraReady] = useState(false);
  const [cameraError, setCameraError] = useState("");
  const [cameraBusy, setCameraBusy] = useState(false);
  const [cameraStreamVersion, setCameraStreamVersion] = useState(0);
  const [gimbalPan, setGimbalPan] = useState(0);
  const [gimbalTilt, setGimbalTilt] = useState(0);
  const [perceptionBusy, setPerceptionBusy] = useState(false);
  const manualTimer = useRef<number | null>(null);
  const gimbalTimer = useRef<number | null>(null);
  const gimbalTarget = useRef({ pan: 0, tilt: 0 });
  const gimbalAdjusting = useRef(false);

  const clearManualTimer = useCallback(() => {
    if (manualTimer.current !== null) {
      window.clearInterval(manualTimer.current);
      manualTimer.current = null;
    }
  }, []);

  useEffect(() => {
    const updateClock = () => setNow(new Date());
    updateClock();
    const timer = window.setInterval(updateClock, 1000);
    return () => window.clearInterval(timer);
  }, []);

  useEffect(() => {
    const timer = window.setTimeout(() => {
      try {
        const storedMaps = window.localStorage.getItem("patrol_robot_maps_v1");
        const storedActiveId = window.localStorage.getItem("patrol_robot_active_map_v1");
        if (storedMaps) {
          const parsed = JSON.parse(storedMaps) as PatrolMap[];
          if (Array.isArray(parsed) && parsed.length && parsed.every((map) => map?.id && map?.bounds && Array.isArray(map.objects) && Array.isArray(map.waypoints))) {
            const nextActive = parsed.find((map) => map.id === storedActiveId) ?? parsed[0];
            setMaps(parsed);
            setActiveMapId(nextActive.id);
            setSelected(nextActive.waypoints[0]?.id ?? 0);
          }
        }
      } catch {
        window.localStorage.removeItem("patrol_robot_maps_v1");
      }
      setMapStorageReady(true);
    }, 0);
    return () => window.clearTimeout(timer);
  }, []);

  useEffect(() => {
    if (!mapStorageReady) return;
    window.localStorage.setItem("patrol_robot_maps_v1", JSON.stringify(maps));
    window.localStorage.setItem("patrol_robot_active_map_v1", activeMapId);
  }, [activeMapId, mapStorageReady, maps]);

  useEffect(() => () => {
    clearManualTimer();
    if (gimbalTimer.current !== null) window.clearTimeout(gimbalTimer.current);
  }, [clearManualTimer]);

  useEffect(() => {
    if (!apiBase) return;
    let active = true;
    const poll = async () => {
      try {
        const response = await fetch(`${apiBase}/api/status`, { cache: "no-store" });
        if (!response.ok) throw new Error("status");
        const status = await response.json();
        if (!active) return;
        setConnected(true);
        const camera = status.camera ?? {
          enabled: false, ok: false, frames: 0, width: 0, height: 0, fps: 0,
          stream_fps: 12, last_frame_age: null, topic: "/camera/color/image_raw",
          pan_deg: 0, tilt_deg: 0, pan_target_deg: 0, tilt_target_deg: 0, gimbal_ok: false,
        };
        const perception = status.perception ?? {
          mode: "fusion", lidar_enabled: true, camera_enabled: true,
          lidar_ok: Boolean(status.lidar_ok), camera_ok: false,
          transitioning: false, gimbal_locked: false, safety_ok: true,
          camera_points: 0, active_sources: [], last_camera_cloud_age: null,
        };
        const mapStatus = status.map ?? {
          active_id: "pipeline-demo", active_name: "管廊综合测试区", transitioning: false, localization_ready: true, error: null,
        };
        setCameraEnabled(Boolean(camera.enabled));
        setPerceptionBusy(Boolean(perception.transitioning));
        if (!gimbalAdjusting.current) {
          const panTarget = Number(camera.pan_target_deg ?? camera.pan_deg ?? 0);
          const tiltTarget = Number(camera.tilt_target_deg ?? camera.tilt_deg ?? 0);
          setGimbalPan(panTarget);
          setGimbalTilt(tiltTarget);
          gimbalTarget.current = { pan: panTarget, tilt: tiltTarget };
        }
        setTelemetry({
          speed: status.speed ?? 0, x: status.x ?? 0, y: status.y ?? 0, yaw: status.yaw ?? 0,
          lidar_ok: Boolean(status.lidar_ok),
          patrol: status.patrol ?? { state: "UNKNOWN", current_index: 0, current_waypoint: "等待任务", waypoint_count: 0 },
          camera,
          perception,
          map: mapStatus,
        });
        setRunning(Boolean(status.patrol?.running));
      } catch {
        if (active) setConnected(false);
      }
    };
    poll();
    const timer = window.setInterval(poll, 1000);
    return () => { active = false; window.clearInterval(timer); };
  }, [apiBase]);

  const heading = useMemo(() => {
    const degrees = (telemetry.yaw * 180 / Math.PI + 360) % 360;
    const directions = ["东", "东北", "北", "西北", "西", "西南", "南", "东南"];
    return { degrees, label: directions[Math.round(degrees / 45) % 8] };
  }, [telemetry.yaw]);
  const timeText = now?.toLocaleTimeString("zh-CN", {
    hour: "2-digit", minute: "2-digit", second: "2-digit", hour12: false,
  }) ?? "--:--:--";
  const dateText = now?.toLocaleDateString("zh-CN", {
    year: "numeric", month: "2-digit", day: "2-digit", weekday: "short",
  }).replaceAll("/", " / ") ?? "---- / -- / --";

  function changeMapZoom(delta: number) {
    setMapZoom((current) => Math.max(0.75, Math.min(2, Number((current + delta).toFixed(2)))));
  }

  async function startManual(linear: number, angular: number, label: string) {
    clearManualTimer();
    setManualActive(label);
    if (running) {
      await send("/api/patrol/stop");
      setRunning(false);
    }
    const transmit = () => void send("/api/control/manual", { linear, angular });
    transmit();
    manualTimer.current = window.setInterval(transmit, 100);
  }

  function stopManual() {
    clearManualTimer();
    setManualActive("已停车");
    void send("/api/control/manual", { linear: 0, angular: 0 });
  }

  function notify(message: string) {
    setToast(message);
    window.setTimeout(() => setToast("所有系统运行正常"), 2600);
  }

  async function send(path: string, body: object = {}) {
    try {
      const response = await fetch(`${apiBase}${path}`, {
        method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body),
      });
      if (!response.ok) throw new Error("request");
      return true;
    } catch {
      setConnected(false);
      notify("无法连接车辆网关，请检查 Ubuntu 地址和端口 8765");
      return false;
    }
  }

  async function togglePatrol() {
    const next = !running;
    if (next) {
      if (telemetry.map.transitioning || telemetry.map.localization_ready === false) {
        notify("地图定位尚未稳定，请稍候再开始巡检");
        return;
      }
      if (!(await saveWaypoints(waypoints, patrolLoops))) return;
    }
    if (await send(`/api/patrol/${next ? "start" : "stop"}`)) {
      setRunning(next);
      notify(next ? "巡检启动命令已发送" : "巡检停止命令已发送");
    }
  }

  async function changePerceptionMode(mode: PerceptionMode) {
    if (perceptionBusy || (mode === telemetry.perception.mode && !telemetry.perception.error)) return;
    setPerceptionBusy(true);
    if (running) {
      await send("/api/patrol/stop");
      setRunning(false);
    }
    if (await send("/api/perception/mode", { mode })) {
      setTelemetry((current) => ({
        ...current,
        perception: {
          ...current.perception,
          mode,
          lidar_enabled: mode !== "camera",
          camera_enabled: mode !== "lidar",
          gimbal_locked: mode === "camera",
          transitioning: true,
          error: null,
        },
      }));
      if (mode === "camera") {
        setGimbalPan(0);
        setGimbalTilt(0);
        gimbalTarget.current = { pan: 0, tilt: 0 };
      }
      notify(mode === "lidar" ? "正在切换为雷达感知" : mode === "camera" ? "正在切换为视觉感知" : "正在开启雷达与视觉融合");
    } else {
      setPerceptionBusy(false);
    }
  }

  async function saveWaypoints(next = waypoints, loopCount = patrolLoops, routeMap = activeMap) {
    if (!next.length) {
      notify("路线未同步：请先在地图上添加至少一个巡检点");
      return false;
    }
    const candidateMap = { ...routeMap, waypoints: next };
    const safetyIssues = validatePatrolWaypoints(candidateMap);
    if (safetyIssues.length) {
      const summary = safetyIssues.slice(0, 2).map(({ waypoint, reason }) => `${waypoint.name}：${reason}`).join("；");
      notify(`路线未同步：${summary}${safetyIssues.length > 2 ? `；另有 ${safetyIssues.length - 2} 个问题` : ""}`);
      return false;
    }
    const saved = await send("/api/navigation/waypoints", {
      frame_id: "map",
      loop_count: Math.max(1, Math.min(1000, Math.round(loopCount))),
      waypoints: next.map(({name,x,y,dwell}) => ({name,x,y,yaw:0,dwell})),
    });
    if (saved) {
      notify("巡检路线已同步到 ROS 2");
    }
    return saved;
  }

  function updateMap(next: PatrolMap) {
    setMaps((current) => current.map((map) => map.id === next.id ? next : map));
  }

  function updateWaypoints(next: Waypoint[]) {
    updateMap({ ...activeMap, waypoints: next, updatedAt: new Date().toISOString() });
  }

  async function activateMap(next: PatrolMap) {
    if (running) {
      await send("/api/patrol/stop");
      setRunning(false);
    }
    setActiveMapId(next.id);
    setSelected(next.waypoints[0]?.id ?? 0);
    setMapMode("3d");
    if (!next.waypoints.length) {
      notify(`“${next.name}”尚未设置巡检点，请编辑路线后再应用到车辆`);
      return;
    }
    const safetyIssues = validatePatrolWaypoints(next);
    if (safetyIssues.length) {
      notify(`“${next.name}”包含不安全巡检点，请调整后再应用到车辆`);
      return;
    }
    if (connected) {
      const applied = await send("/api/maps/activate", mapToRobotPayload(next));
      if (applied) await saveWaypoints(next.waypoints, patrolLoops, next);
    } else {
      notify(`已在本机切换到“${next.name}”，连接车辆后可应用到 ROS 2`);
    }
  }

  function addMaps(nextMaps: PatrolMap[]) {
    if (!nextMaps.length) return;
    setMaps((current) => [...current, ...nextMaps]);
    void activateMap(nextMaps[nextMaps.length - 1]);
  }

  function duplicateMap(source: PatrolMap) {
    const timestamp = new Date().toISOString();
    const unique = timestamp.replace(/\D/g, "");
    const duplicate: PatrolMap = {
      ...source,
      id: `${source.id}-copy-${unique}`,
      name: `${source.name} · 副本`,
      source: source.source === "preset" ? "generated" : source.source,
      objects: source.objects.map((object, index) => ({ ...object, id: `${object.id}-${unique}-${index}` })),
      waypoints: source.waypoints.map((waypoint) => ({ ...waypoint })),
      createdAt: timestamp,
      updatedAt: timestamp,
    };
    addMaps([duplicate]);
    notify("已复制当前地图");
  }

  function deleteMap(id: string) {
    if (maps.length <= 1) return;
    const nextMaps = maps.filter((map) => map.id !== id);
    setMaps(nextMaps);
    if (id === activeMapId) void activateMap(nextMaps[0]);
    notify("地图已从场景库删除");
  }

  function addWaypoint() {
    const id = Math.max(...waypoints.map((item) => item.id), 0) + 1;
    const centerX = activeMap.bounds.minX + activeMap.bounds.width / 2;
    const centerY = activeMap.bounds.minY + activeMap.bounds.height / 2;
    const safePosition = findNearestSafeWaypointPosition(activeMap, centerX, centerY);
    if (!safePosition) {
      notify("当前地图没有找到可用的巡检点位置，请先移除部分障碍物");
      return;
    }
    const point = { id, name: `新巡检点 ${id}`, ...safePosition, dwell: 3 };
    updateWaypoints([...waypoints, point]);
    setSelected(id);
    saveWaypoints([...waypoints, point]);
  }

  function removeWaypoint(id: number) {
    if (waypoints.length <= 1) return;
    const next = waypoints.filter((item) => item.id !== id);
    updateWaypoints(next);
    if (selected === id) setSelected(next[0].id);
    saveWaypoints(next);
  }

  async function toggleCamera() {
    if (cameraBusy) return;
    const next = !cameraEnabled;
    setCameraBusy(true);
    if (await send("/api/camera/enable", { enabled: next })) {
      setCameraEnabled(next);
      setCameraReady(false);
      setCameraError("");
      setCameraStreamVersion((version) => version + 1);
      if (next) setMapMode("camera");
      notify(next ? "云台摄像头正在开启" : "摄像头画面已关闭");
    }
    setCameraBusy(false);
  }

  function retryCameraStream() {
    setCameraReady(false);
    setCameraError("");
    setCameraStreamVersion((version) => version + 1);
  }

  function commandGimbal(panValue: number, tiltValue: number, immediate = false) {
    if (telemetry.perception.gimbal_locked) return;
    const pan = Math.max(-CAMERA_LIMITS.pan, Math.min(panValue, CAMERA_LIMITS.pan));
    const tilt = Math.max(-CAMERA_LIMITS.tiltDown, Math.min(tiltValue, CAMERA_LIMITS.tiltUp));
    setGimbalPan(pan);
    setGimbalTilt(tilt);
    gimbalTarget.current = { pan, tilt };

    const transmit = () => {
      const target = gimbalTarget.current;
      void send("/api/camera/gimbal", target);
    };
    if (immediate) {
      if (gimbalTimer.current !== null) window.clearTimeout(gimbalTimer.current);
      gimbalTimer.current = null;
      transmit();
    } else if (gimbalTimer.current === null) {
      gimbalTimer.current = window.setTimeout(() => {
        gimbalTimer.current = null;
        transmit();
      }, 80);
    }
  }

  function finishGimbalAdjustment() {
    gimbalAdjusting.current = false;
    commandGimbal(gimbalTarget.current.pan, gimbalTarget.current.tilt, true);
  }

  function nudgeGimbal(panDelta: number, tiltDelta: number) {
    commandGimbal(gimbalPan + panDelta, gimbalTilt + tiltDelta, true);
  }

  const cameraStreamUrl = `${apiBase}/api/camera/stream?v=${cameraStreamVersion}`;
  // Some browsers keep an MJPEG request open without firing a normal image
  // load event. The gateway heartbeat is a second confirmation that frames exist.
  const cameraShowing = cameraReady || (telemetry.camera.ok && !cameraError);

  return (
    <main className="app-shell">
      <aside className="sidebar">
        <div className="brand">
          <div className="brand-mark">巡</div>
          <div><strong>巡检智控</strong><span>ROBOT FLEET</span></div>
        </div>

        <nav aria-label="主导航">
          <button className={`nav-item ${activeSection === "control" && mapMode !== "camera" ? "active" : ""}`} onClick={() => { setActiveSection("control"); setMapMode("2d"); }}><i>⌁</i> 车辆控制</button>
          <button className="nav-item"><i>⌖</i> 巡检任务 <b>{waypoints.length}</b></button>
          <button className={`nav-item ${activeSection === "maps" ? "active" : ""}`} onClick={() => setActiveSection("maps")}><i>◫</i> 地图管理 <b>{maps.length}</b></button>
          <button className={`nav-item ${activeSection === "control" && mapMode === "camera" ? "active" : ""}`} onClick={() => { setActiveSection("control"); setMapMode("camera"); }}><i>◉</i> 设备监控</button>
          <button className="nav-item"><i>⚙</i> 系统设置</button>
        </nav>

        <div className="robot-card">
          <div className="robot-thumb"><span></span><b></b></div>
          <div><small>当前车辆</small><strong>巡检车 · 01</strong><em>● 在线</em></div>
          <button aria-label="切换车辆">⌄</button>
        </div>
      </aside>

      <section className="workspace">
        <header className="topbar">
          <div><p>{activeSection === "maps" ? "地图场景工作台" : "车辆控制台"}</p><small>{activeSection === "maps" ? `当前地图 · ${activeMap.name}` : "实时控制与任务配置"}</small></div>
          <div className="status-cluster">
            <span className={`status-pill ${connected ? "" : "offline"}`} title={apiBase}><i></i>{connected ? toast : "车辆网关未连接"}</span>
            <span className="clock">{timeText} <small>{dateText}</small></span>
            <button className="icon-button" aria-label="通知">♧<b>2</b></button>
            <div className="avatar">QC</div>
          </div>
        </header>

        <div className={`content ${activeSection === "maps" ? "map-content" : ""}`}>
          {activeSection === "maps" ? (
            <MapManagement
              maps={maps}
              activeMapId={activeMapId}
              robotX={telemetry.x}
              robotY={telemetry.y}
              robotYaw={telemetry.yaw}
              onActivate={(map) => void activateMap(map)}
              onChange={updateMap}
              onAdd={addMaps}
              onDuplicate={duplicateMap}
              onDelete={deleteMap}
              onNotice={notify}
              connected={connected}
              runtimeMap={telemetry.map}
            />
          ) : (<>
          <section className="hero-panel">
            <div className={`map-area view-${mapMode}`} onWheel={(event) => { event.preventDefault(); changeMapZoom(event.deltaY < 0 ? 0.1 : -0.1); }}>
              <div className="map-mode-switch" role="group" aria-label="地图显示模式">
                <button className={mapMode === "2d" ? "active" : ""} onClick={() => setMapMode("2d")}>2D 地图</button>
                <button className={mapMode === "3d" ? "active" : ""} onClick={() => setMapMode("3d")}>3D 场景</button>
                <button className={mapMode === "camera" ? "active" : ""} onClick={() => setMapMode("camera")}><i className={telemetry.camera.ok ? "camera-dot online" : "camera-dot"}></i>摄像画面</button>
              </div>
              {mapMode === "camera" ? (
                <div className="camera-view">
                  {cameraEnabled ? (
                    <>
                      <img
                        key={cameraStreamVersion}
                        src={cameraStreamUrl}
                        alt="巡检车两轴云台摄像头实时画面"
                        onLoad={() => { setCameraReady(true); setCameraError(""); }}
                        onError={() => { setCameraReady(false); setCameraError("暂时没有收到摄像头画面"); }}
                      />
                      {!cameraShowing && !cameraError && (
                        <div className="camera-loading" role="status"><span></span><strong>正在连接云台摄像头</strong><small>等待 ROS 2 图像数据</small></div>
                      )}
                      {cameraError && (
                        <div className="camera-loading camera-failed"><b>!</b><strong>{cameraError}</strong><small>请确认 3D 仿真与相机话题正在运行</small><button onClick={retryCameraStream}>重新连接</button></div>
                      )}
                      <div className="camera-hud">
                        <span className={telemetry.camera.ok ? "live" : "waiting"}><i></i>{telemetry.camera.ok ? "LIVE" : "等待信号"}</span>
                        <small>{telemetry.camera.width || 640} × {telemetry.camera.height || 480}　·　{telemetry.camera.fps?.toFixed(1) || "0.0"} FPS</small>
                        <button onClick={toggleCamera} disabled={cameraBusy}>{cameraBusy ? "处理中" : "关闭摄像头"}</button>
                      </div>
                      <div className="gimbal-control" aria-label="云台控制器">
                        <div className="gimbal-heading">
                          <span><i className={telemetry.camera.gimbal_ok ? "online" : ""}></i>两轴云台</span>
                          <strong>{telemetry.perception.gimbal_locked ? "视觉导航 · 正前方锁定" : `水平 ${gimbalPan.toFixed(0)}°　俯仰 ${gimbalTilt.toFixed(0)}°`}</strong>
                        </div>
                        <div className="gimbal-row">
                          <button onClick={() => nudgeGimbal(-10, 0)} disabled={!connected || telemetry.perception.gimbal_locked} aria-label="摄像头向左转动">‹</button>
                          <input
                            aria-label="摄像头水平角度"
                            type="range"
                            min={-CAMERA_LIMITS.pan}
                            max={CAMERA_LIMITS.pan}
                            step="1"
                            value={gimbalPan}
                            disabled={!connected || telemetry.perception.gimbal_locked}
                            onPointerDown={() => { gimbalAdjusting.current = true; }}
                            onKeyDown={() => { gimbalAdjusting.current = true; }}
                            onChange={(event) => commandGimbal(Number(event.target.value), gimbalTilt)}
                            onPointerUp={finishGimbalAdjustment}
                            onPointerCancel={finishGimbalAdjustment}
                            onKeyUp={finishGimbalAdjustment}
                            onBlur={finishGimbalAdjustment}
                          />
                          <button onClick={() => nudgeGimbal(10, 0)} disabled={!connected || telemetry.perception.gimbal_locked} aria-label="摄像头向右转动">›</button>
                        </div>
                        <div className="gimbal-row">
                          <button onClick={() => nudgeGimbal(0, -5)} disabled={!connected || telemetry.perception.gimbal_locked} aria-label="摄像头向下转动">⌄</button>
                          <input
                            aria-label="摄像头俯仰角度"
                            type="range"
                            min={-CAMERA_LIMITS.tiltDown}
                            max={CAMERA_LIMITS.tiltUp}
                            step="1"
                            value={gimbalTilt}
                            disabled={!connected || telemetry.perception.gimbal_locked}
                            onPointerDown={() => { gimbalAdjusting.current = true; }}
                            onKeyDown={() => { gimbalAdjusting.current = true; }}
                            onChange={(event) => commandGimbal(gimbalPan, Number(event.target.value))}
                            onPointerUp={finishGimbalAdjustment}
                            onPointerCancel={finishGimbalAdjustment}
                            onKeyUp={finishGimbalAdjustment}
                            onBlur={finishGimbalAdjustment}
                          />
                          <button onClick={() => nudgeGimbal(0, 5)} disabled={!connected || telemetry.perception.gimbal_locked} aria-label="摄像头向上转动">⌃</button>
                        </div>
                        <button className="gimbal-center" onClick={() => commandGimbal(0, 0, true)} disabled={!connected || telemetry.perception.gimbal_locked}>{telemetry.perception.gimbal_locked ? "视觉导航正在使用正前方视角" : "回到正前方"}</button>
                      </div>
                      <div className="camera-caption"><strong>CAM-01</strong><span>RGB-D 云台视角</span><em>已接收 {telemetry.camera.frames} 帧</em></div>
                    </>
                  ) : (
                    <div className="camera-offline">
                      <div className="camera-lens"><i></i></div>
                      <span>CAM-01 · 两轴 RGB-D 云台摄像头</span>
                      <h2>摄像头画面未开启</h2>
                      <p>开启后可实时查看周围环境，并在画面内控制水平与俯仰方向；关闭网页画面可减少视频编码和网络占用。</p>
                      <button onClick={toggleCamera} disabled={!connected || cameraBusy}>{cameraBusy ? "正在开启…" : "开启摄像头"}</button>
                    </div>
                  )}
                </div>
              ) : mapMode === "3d" ? (
                <Industrial3DMap map={activeMap} robotX={telemetry.x} robotY={telemetry.y} robotYaw={telemetry.yaw} zoom={mapZoom} selected={selected} onSelect={setSelected} />
              ) : <Industrial2DMap map={activeMap} robotX={telemetry.x} robotY={telemetry.y} robotYaw={telemetry.yaw} zoom={mapZoom} selected={selected} onSelect={setSelected} />}
              {mapMode !== "camera" && (
                <>
                  <div className="map-tools">
                    <button onClick={() => changeMapZoom(0.25)} disabled={mapZoom >= 2} aria-label="放大地图" title="放大地图">＋</button>
                    <button onClick={() => changeMapZoom(-0.25)} disabled={mapZoom <= 0.75} aria-label="缩小地图" title="缩小地图">−</button>
                    <button onClick={() => setMapZoom(1)} aria-label="恢复默认缩放" title="恢复默认缩放">⌖</button>
                  </div>
                  <div className="map-scale"><strong>{Math.round(mapZoom * 100)}%</strong><span>{(10 / mapZoom).toFixed(mapZoom === 1 ? 0 : 1)} m</span></div>
                </>
              )}
            </div>

            <aside className="telemetry">
              <div className="eyebrow"><span>实时状态</span><i>LIVE</i></div>
              <div className="speed-gauge">
                <div><strong>{connected ? telemetry.speed.toFixed(1) : "0.0"}</strong><small>m/s</small></div>
              </div>
              <p className="drive-state"><i></i>{running ? telemetry.patrol.returning_home ? "本圈结束 · 正在返回出发点" : `前往：${telemetry.patrol.current_waypoint}` : telemetry.map.transitioning ? "地图定位校准中" : "车辆已就绪"}</p>
              <div className="stat-grid">
                <div><small>电池电量</small><strong>86<span>%</span></strong><progress value="86" max="100" /></div>
                <div><small>激光雷达</small><strong>{telemetry.perception.lidar_enabled ? telemetry.lidar_ok ? "正常" : "无数据" : "未参与"}</strong><span className="signal">▂▄▆█</span></div>
                <div><small>当前巡检点</small><strong>{telemetry.patrol.current_index + 1}<span> / {telemetry.patrol.waypoint_count || waypoints.length}</span></strong></div>
                <div><small>地图坐标</small><strong>{telemetry.x.toFixed(1)}<span>, {telemetry.y.toFixed(1)} m</span></strong></div>
              </div>
              <label className="patrol-loop-control">
                <span>巡检圈数</span>
                <button type="button" onClick={() => setPatrolLoops((value) => Math.max(1, value - 1))} disabled={running || patrolLoops <= 1}>−</button>
                <input aria-label="巡检圈数" type="number" min="1" max="1000" value={patrolLoops} disabled={running} onChange={(event) => setPatrolLoops(Math.max(1, Math.min(1000, Number(event.target.value) || 1)))} />
                <button type="button" onClick={() => setPatrolLoops((value) => Math.min(1000, value + 1))} disabled={running || patrolLoops >= 1000}>＋</button>
                <small>已完成 {telemetry.patrol.completed_loops ?? 0} / {telemetry.patrol.loop_count ?? patrolLoops}</small>
              </label>
              <button className={`primary-action ${running ? "stop" : ""}`} onClick={togglePatrol} disabled={!running && (telemetry.map.transitioning || telemetry.map.localization_ready === false)}>
                {running ? "■  停止巡检" : "▶  开始巡检"}
              </button>
              <button className="emergency" onClick={async () => { await send("/api/control/emergency-stop"); setRunning(false); notify("车辆已紧急停止"); }}>紧急停止</button>
            </aside>
          </section>

          <section className="lower-grid">
            <div className="panel perception-panel">
              <div className="panel-heading">
                <div><span className="heading-icon perception-icon">◈</span><div><h2>导航感知模式</h2><p>切换雷达与 RGB-D 相机参与避障的方式</p></div></div>
                <span className={`perception-state ${connected && telemetry.perception.safety_ok ? "healthy" : "fault"}`}>{!connected ? "等待车辆" : telemetry.perception.transitioning ? "切换中" : telemetry.perception.safety_ok ? "感知正常" : "已安全停车"}</span>
              </div>
              <div className="perception-layout">
                <div className="perception-modes" role="group" aria-label="导航感知模式">
                  {PERCEPTION_MODES.map((mode) => (
                    <button
                      key={mode.id}
                      className={telemetry.perception.mode === mode.id ? "active" : ""}
                      onClick={() => changePerceptionMode(mode.id)}
                      disabled={!connected || perceptionBusy}
                    >
                      <i>{mode.icon}</i><span><strong>{mode.name}</strong><small>{mode.description}</small></span>
                    </button>
                  ))}
                </div>
                <div className="perception-sensors">
                  <div className={`${telemetry.perception.lidar_enabled ? "enabled" : ""} ${telemetry.perception.lidar_ok ? "online" : ""}`}>
                    <span className="sensor-symbol lidar-symbol"><i></i></span>
                    <p><strong>激光雷达</strong><small>{telemetry.perception.lidar_enabled ? telemetry.perception.lidar_ok ? "导航数据正常" : "等待扫描数据" : "未参与导航"}</small></p>
                    <em>{telemetry.perception.lidar_enabled ? telemetry.perception.lidar_ok ? "ON" : "WAIT" : "OFF"}</em>
                  </div>
                  <div className={`${telemetry.perception.camera_enabled ? "enabled" : ""} ${telemetry.perception.camera_ok ? "online" : ""}`}>
                    <span className="sensor-symbol camera-symbol"><i></i></span>
                    <p><strong>RGB-D 相机</strong><small>{telemetry.perception.camera_enabled ? telemetry.perception.camera_ok ? `${telemetry.perception.camera_points.toLocaleString()} 点 · 正常` : "等待深度点云" : "未参与导航"}</small></p>
                    <em>{telemetry.perception.camera_enabled ? telemetry.perception.camera_ok ? "ON" : "WAIT" : "OFF"}</em>
                  </div>
                </div>
              </div>
              <p className={`perception-note ${telemetry.perception.error ? "error" : ""}`}>{telemetry.perception.error || (telemetry.perception.mode === "camera" ? "视觉模式会锁定云台正前方；如果深度点云中断，车辆将自动停车。" : telemetry.perception.mode === "fusion" ? "推荐生产模式：任一传感器短暂失效时，另一传感器仍可维持避障。" : "雷达模式不使用相机点云参与导航，网页视频仍可单独开启。")}</p>
            </div>

            <div className="panel speed-panel">
              <div className="panel-heading"><div><span className="heading-icon">⇄</span><div><h2>行驶参数</h2><p>调节车辆运行速度</p></div></div><span className="saved">✓ 自动保存</span></div>
              <div className="speed-control">
                <div className="speed-value"><small>最高速度限制</small><strong>{speed.toFixed(1)} <span>m/s</span></strong></div>
                <input aria-label="最高速度限制" type="range" min="0.1" max="1.5" step="0.1" value={speed} onChange={(e) => setSpeed(Number(e.target.value))} onPointerUp={() => send("/api/config/speed", { linear: speed, angular: 0.8 })} />
                <div className="range-labels"><span>0.1<br/><small>精细</small></span><span>0.8<br/><small>标准</small></span><span>1.5<br/><small>快速</small></span></div>
              </div>
              <div className="quick-speeds">
                {[0.3, 0.6, 1.0].map((item) => <button key={item} className={speed === item ? "active" : ""} onClick={() => { setSpeed(item); send("/api/config/speed", { linear: item, angular: 0.8 }); }}>{item === 0.3 ? "精细" : item === 0.6 ? "标准" : "快速"}<strong>{item.toFixed(1)} m/s</strong></button>)}
              </div>
            </div>

            <div className="panel manual-panel">
              <div className="panel-heading"><div><span className="heading-icon manual-icon">✥</span><div><h2>人工控制</h2><p>按住方向键移动 · 松手立即停车</p></div></div><span className={`manual-state ${manualActive !== "未接管" ? "active" : ""}`}>{manualActive}</span></div>
              <div className="manual-layout">
                <div className="drive-pad" onPointerLeave={stopManual}>
                  <button className="drive-forward" aria-label="向前" onPointerDown={() => startManual(Math.min(speed, 0.6), 0, "向前") } onPointerUp={stopManual} onPointerCancel={stopManual}><b>↑</b><small>前进</small></button>
                  <button className="drive-left" aria-label="左转" onPointerDown={() => startManual(0, 0.6, "左转") } onPointerUp={stopManual} onPointerCancel={stopManual}><b>↶</b><small>左转</small></button>
                  <button className="drive-stop" aria-label="停车" onClick={stopManual}>■</button>
                  <button className="drive-right" aria-label="右转" onPointerDown={() => startManual(0, -0.6, "右转") } onPointerUp={stopManual} onPointerCancel={stopManual}><b>↷</b><small>右转</small></button>
                  <button className="drive-back" aria-label="后退" onPointerDown={() => startManual(-Math.min(speed, 0.4), 0, "后退") } onPointerUp={stopManual} onPointerCancel={stopManual}><b>↓</b><small>后退</small></button>
                </div>
                <div className="manual-help"><strong>安全人工接管 · 车头向{heading.label} {heading.degrees.toFixed(0)}°</strong><p>前进/后退沿车身纵向移动，左转/右转为原地旋转。开始操作会自动暂停巡检，脱困后点击“开始巡检”恢复任务。</p><span>前进上限 {Math.min(speed, 0.6).toFixed(1)} m/s · 后退上限 {Math.min(speed, 0.4).toFixed(1)} m/s</span></div>
              </div>
            </div>

            <div className="panel points-panel">
              <div className="panel-heading"><div><span className="heading-icon orange">⌖</span><div><h2>巡检点</h2><p>当前路线 · {waypoints.length} 个点</p></div></div><button className="add-button" onClick={addWaypoint}>＋ 添加巡检点</button></div>
              <div className="point-list">
                {waypoints.map((point, index) => (
                  <div key={point.id} className={`point-row ${selected === point.id ? "active" : ""}`} onClick={() => setSelected(point.id)}>
                    <span className="point-index">{index + 1}</span>
                    <div><strong>{point.name}{index === 0 && <em className="home-point-badge">出发 / 返回</em>}</strong><small>X {point.x.toFixed(1)}　Y {point.y.toFixed(1)}　· 停留 {point.dwell} 秒</small></div>
                    <button aria-label={`删除${point.name}`} onClick={(event) => { event.stopPropagation(); removeWaypoint(point.id); }}>×</button>
                  </div>
                ))}
              </div>
            </div>

            <div className="panel hardware-panel">
              <div className="panel-heading"><div><span className="heading-icon purple">◇</span><div><h2>车辆硬件配置</h2><p>开发模式 · 修改后需重启生效</p></div></div><span className="dev-badge">DEV</span></div>
              <div className="vehicle-config">
                <div className="vehicle-preview"><div className="lidar-stack">{Array.from({length: lidars}).map((_, i) => <i key={i}></i>)}</div><div className="car-body" style={{width: `${88 + (length - 40) * 2}px`, height: `${48 + (height - 12) * 2}px`}}></div><div className="wheels"><i></i><i></i></div><span>{length} × {width} × {height} cm</span></div>
                <div className="config-fields">
                  <label>车辆高度 <span>{height} cm</span><input type="range" min="12" max="35" value={height} onChange={(e) => setHeight(Number(e.target.value))}/></label>
                  <div className="dual-fields"><label>底盘长度<input type="number" value={length} onChange={(e) => setLength(Number(e.target.value))}/><i>cm</i></label><label>底盘宽度<input type="number" value={width} onChange={(e) => setWidth(Number(e.target.value))}/><i>cm</i></label></div>
                  <div className="lidar-field"><span>激光雷达数量</span><div>{[1,2,3,4].map((item) => <button className={lidars === item ? "active" : ""} onClick={() => setLidars(item)} key={item}>{item}</button>)}</div></div>
                  <button className="apply-config" onClick={async () => { if (await send("/api/config/hardware", { height_cm: height, length_cm: length, width_cm: width, lidar_count: lidars })) notify(`硬件配置已提交：${lidars} 个雷达（重启后生效）`); }}>应用硬件配置</button>
                </div>
              </div>
            </div>
          </section>
          </>)}
        </div>
      </section>
    </main>
  );
}

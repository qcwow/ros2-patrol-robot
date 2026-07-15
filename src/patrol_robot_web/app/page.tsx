"use client";

import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { Industrial3DMap } from "./Industrial3DMap";
import { Industrial2DMap } from "./Industrial2DMap";

type Waypoint = { id: number; name: string; x: number; y: number; dwell: number };
type Telemetry = {
  speed: number; x: number; y: number; yaw: number; lidar_ok: boolean;
  patrol: { state: string; current_index: number; current_waypoint: string; waypoint_count: number };
};

const initialWaypoints: Waypoint[] = [
  { id: 1, name: "起点东侧", x: -4.8, y: -3.8, dwell: 2 },
  { id: 2, name: "A区管道北侧", x: -4.6, y: 3.8, dwell: 3 },
  { id: 3, name: "控制柜检查点", x: 0.2, y: 3.2, dwell: 3 },
  { id: 4, name: "B区管道东侧", x: 4.6, y: -1.0, dwell: 3 },
  { id: 5, name: "返回区", x: -5.8, y: -4.0, dwell: 2 },
];

export default function Home() {
  const [speed, setSpeed] = useState(0.6);
  const [height, setHeight] = useState(16);
  const [length, setLength] = useState(52);
  const [width, setWidth] = useState(42);
  const [lidars, setLidars] = useState(1);
  const [running, setRunning] = useState(false);
  const [waypoints, setWaypoints] = useState(initialWaypoints);
  const [selected, setSelected] = useState(3);
  const [toast, setToast] = useState("所有系统运行正常");
  const [connected, setConnected] = useState(false);
  const [telemetry, setTelemetry] = useState<Telemetry>({
    speed: 0, x: -6, y: -4, yaw: 0, lidar_ok: false,
    patrol: { state: "UNKNOWN", current_index: 0, current_waypoint: "等待连接", waypoint_count: 0 },
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
  const [mapMode, setMapMode] = useState<"2d" | "3d">("2d");
  const [manualActive, setManualActive] = useState("未接管");
  const manualTimer = useRef<number | null>(null);

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

  useEffect(() => () => clearManualTimer(), [clearManualTimer]);

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
        setTelemetry({
          speed: status.speed ?? 0, x: status.x ?? 0, y: status.y ?? 0, yaw: status.yaw ?? 0,
          lidar_ok: Boolean(status.lidar_ok),
          patrol: status.patrol ?? { state: "UNKNOWN", current_index: 0, current_waypoint: "等待任务", waypoint_count: 0 },
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
    if (await send(`/api/patrol/${next ? "start" : "stop"}`)) {
      setRunning(next);
      notify(next ? "巡检启动命令已发送" : "巡检停止命令已发送");
    }
  }

  async function saveWaypoints(next = waypoints) {
    if (await send("/api/navigation/waypoints", { frame_id: "map", waypoints: next.map(({name,x,y,dwell}) => ({name,x,y,yaw:0,dwell})) })) {
      notify("巡检路线已同步到 ROS 2");
    }
  }

  function addWaypoint() {
    const id = Math.max(...waypoints.map((item) => item.id), 0) + 1;
    const point = { id, name: `新巡检点 ${id}`, x: 1.5, y: 1.2, dwell: 3 };
    setWaypoints([...waypoints, point]);
    setSelected(id);
    saveWaypoints([...waypoints, point]);
  }

  function removeWaypoint(id: number) {
    if (waypoints.length <= 1) return;
    const next = waypoints.filter((item) => item.id !== id);
    setWaypoints(next);
    if (selected === id) setSelected(next[0].id);
    saveWaypoints(next);
  }

  return (
    <main className="app-shell">
      <aside className="sidebar">
        <div className="brand">
          <div className="brand-mark">巡</div>
          <div><strong>巡检智控</strong><span>ROBOT FLEET</span></div>
        </div>

        <nav aria-label="主导航">
          <button className="nav-item active"><i>⌁</i> 车辆控制</button>
          <button className="nav-item"><i>⌖</i> 巡检任务 <b>{waypoints.length}</b></button>
          <button className="nav-item"><i>◫</i> 地图管理</button>
          <button className="nav-item"><i>◉</i> 设备监控</button>
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
          <div><p>车辆控制台</p><small>实时控制与任务配置</small></div>
          <div className="status-cluster">
            <span className={`status-pill ${connected ? "" : "offline"}`} title={apiBase}><i></i>{connected ? toast : "车辆网关未连接"}</span>
            <span className="clock">{timeText} <small>{dateText}</small></span>
            <button className="icon-button" aria-label="通知">♧<b>2</b></button>
            <div className="avatar">QC</div>
          </div>
        </header>

        <div className="content">
          <section className="hero-panel">
            <div className={`map-area view-${mapMode}`} onWheel={(event) => { event.preventDefault(); changeMapZoom(event.deltaY < 0 ? 0.1 : -0.1); }}>
              <div className="map-mode-switch" role="group" aria-label="地图显示模式">
                <button className={mapMode === "2d" ? "active" : ""} onClick={() => setMapMode("2d")}>2D 地图</button>
                <button className={mapMode === "3d" ? "active" : ""} onClick={() => setMapMode("3d")}>3D 场景</button>
              </div>
              {mapMode === "3d" ? (
                <Industrial3DMap robotX={telemetry.x} robotY={telemetry.y} robotYaw={telemetry.yaw} zoom={mapZoom} waypoints={waypoints} selected={selected} />
              ) : <Industrial2DMap robotX={telemetry.x} robotY={telemetry.y} robotYaw={telemetry.yaw} zoom={mapZoom} waypoints={waypoints} selected={selected} onSelect={setSelected} />}
              <div className="map-tools">
                <button onClick={() => changeMapZoom(0.25)} disabled={mapZoom >= 2} aria-label="放大地图" title="放大地图">＋</button>
                <button onClick={() => changeMapZoom(-0.25)} disabled={mapZoom <= 0.75} aria-label="缩小地图" title="缩小地图">−</button>
                <button onClick={() => setMapZoom(1)} aria-label="恢复默认缩放" title="恢复默认缩放">⌖</button>
              </div>
              <div className="map-scale"><strong>{Math.round(mapZoom * 100)}%</strong><span>{(10 / mapZoom).toFixed(mapZoom === 1 ? 0 : 1)} m</span></div>
            </div>

            <aside className="telemetry">
              <div className="eyebrow"><span>实时状态</span><i>LIVE</i></div>
              <div className="speed-gauge">
                <div><strong>{connected ? telemetry.speed.toFixed(1) : "0.0"}</strong><small>m/s</small></div>
              </div>
              <p className="drive-state"><i></i>{running ? `前往：${telemetry.patrol.current_waypoint}` : "车辆已就绪"}</p>
              <div className="stat-grid">
                <div><small>电池电量</small><strong>86<span>%</span></strong><progress value="86" max="100" /></div>
                <div><small>激光雷达</small><strong>{telemetry.lidar_ok ? "正常" : "无数据"}</strong><span className="signal">▂▄▆█</span></div>
                <div><small>当前巡检点</small><strong>{telemetry.patrol.current_index + 1}<span> / {telemetry.patrol.waypoint_count || waypoints.length}</span></strong></div>
                <div><small>地图坐标</small><strong>{telemetry.x.toFixed(1)}<span>, {telemetry.y.toFixed(1)} m</span></strong></div>
              </div>
              <button className={`primary-action ${running ? "stop" : ""}`} onClick={togglePatrol}>
                {running ? "■  停止巡检" : "▶  开始巡检"}
              </button>
              <button className="emergency" onClick={async () => { await send("/api/control/emergency-stop"); setRunning(false); notify("车辆已紧急停止"); }}>紧急停止</button>
            </aside>
          </section>

          <section className="lower-grid">
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
                    <div><strong>{point.name}</strong><small>X {point.x.toFixed(1)}　Y {point.y.toFixed(1)}　· 停留 {point.dwell} 秒</small></div>
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
        </div>
      </section>
    </main>
  );
}

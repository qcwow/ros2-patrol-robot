"use client";

/* eslint-disable @next/next/no-img-element -- the robot camera is an MJPEG stream. */

import {
  useCallback,
  useEffect,
  useMemo,
  useRef,
  useState,
  type MouseEvent as ReactMouseEvent,
  type MutableRefObject,
  type PointerEvent as ReactPointerEvent,
} from "react";
import {
  VoxelMapScene,
  type LiveVoxelSnapshot,
} from "./AutonomousMappingWorkspace";
import type { PatrolMap } from "./mapTypes";

type MappingRuntime = {
  state: string;
  detail: string;
  mode: string;
  enabled: boolean;
  goals_reached: number;
  goals_failed: number;
  frontier_clusters: number;
  blacklisted_goals: number;
  duration_seconds: number;
  coverage: number;
  known_cells: number;
  total_cells: number;
  autonomous_available?: boolean;
  save_error?: string | null;
  saved_map?: SavedMap | null;
};

type LiveMapSnapshot = {
  ok: boolean;
  frame_id: string;
  revision: number;
  width: number;
  height: number;
  resolution: number;
  origin: { x: number; y: number };
  encoding: "rle-int8";
  runs: number[];
  known_cells: number;
  occupied_cells: number;
  total_cells: number;
  coverage: number;
  robot: { x: number; y: number; yaw: number };
  mapping: MappingRuntime;
  camera: {
    enabled: boolean;
    ok: boolean;
    frames: number;
    fps: number;
  };
};

type SavedMap = {
  id: string;
  name: string;
  created_at: string;
  size_bytes: number;
  resolution: number;
  width: number;
  height: number;
  coverage: number;
  has_2d?: boolean;
  has_3d?: boolean;
  voxel_size_bytes?: number;
  editor_map?: PatrolMap;
};

type Props = {
  apiBase: string;
  connected: boolean;
  onGatewayChange: (value: string) => void;
  onSavedMapsChange?: (maps: PatrolMap[]) => void;
  onEditSavedMap?: (mapId: string) => void;
  operation: {
    owner: "idle" | "patrol" | "mapping" | "map";
    locked: boolean;
    detail: string;
  };
  onReturnToNavigation?: () => void;
};

type ManualDrivePadProps = {
  disabled: boolean;
  onDrive: (linear: number, angular: number) => void;
};

const EMPTY_MAPPING: MappingRuntime = {
  state: "IDLE",
  detail: "等待开始新的建图任务",
  mode: "idle",
  enabled: false,
  goals_reached: 0,
  goals_failed: 0,
  frontier_clusters: 0,
  blacklisted_goals: 0,
  duration_seconds: 0,
  coverage: 0,
  known_cells: 0,
  total_cells: 0,
};

const STATE_COPY: Record<string, { label: string; tone: string }> = {
  IDLE: { label: "空闲中", tone: "idle" },
  MAPPING: { label: "手动建图中", tone: "active" },
  STARTING: { label: "自主探索启动中", tone: "active" },
  EXPLORING: { label: "自主探索建图中", tone: "active" },
  NAVIGATING: { label: "自主探索建图中", tone: "active" },
  PAUSED: { label: "建图已暂停", tone: "paused" },
  SAVING: { label: "地图保存中", tone: "saving" },
  SAVED: { label: "地图已保存", tone: "saved" },
  COMPLETED: { label: "自主探索完成", tone: "saved" },
  COMPLETED_WITH_UNREACHABLE: {
    label: "探索安全结束",
    tone: "paused",
  },
  ERROR: { label: "建图异常", tone: "error" },
  DEPLOYING: { label: "地图部署中", tone: "saving" },
  DEPLOYED: { label: "地图已部署", tone: "saved" },
  RESETTING: { label: "正在重置会话", tone: "saving" },
};

function formatDuration(totalSeconds: number) {
  const seconds = Math.max(0, Math.floor(totalSeconds));
  return [Math.floor(seconds / 3600), Math.floor((seconds % 3600) / 60), seconds % 60]
    .map((value) => String(value).padStart(2, "0"))
    .join(":");
}

function formatDate(value: string) {
  if (!value) return "刚刚";
  return new Intl.DateTimeFormat("zh-CN", {
    month: "2-digit",
    day: "2-digit",
    hour: "2-digit",
    minute: "2-digit",
    hour12: false,
  }).format(new Date(value));
}

function decodeRuns(snapshot: LiveMapSnapshot) {
  const cells = new Int8Array(snapshot.total_cells);
  let target = 0;
  for (let index = 0; index + 1 < snapshot.runs.length; index += 2) {
    const value = snapshot.runs[index];
    const count = snapshot.runs[index + 1];
    cells.fill(value, target, Math.min(target + count, cells.length));
    target += count;
    if (target >= cells.length) break;
  }
  return cells;
}

function LiveOccupancyCanvas({
  snapshot,
  canvasRef,
}: {
  snapshot: LiveMapSnapshot | null;
  canvasRef: MutableRefObject<HTMLCanvasElement | null>;
}) {
  const hostRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    const host = hostRef.current;
    if (!canvas || !host) return;

    const draw = () => {
      const rect = host.getBoundingClientRect();
      const pixelRatio = Math.min(window.devicePixelRatio || 1, 2);
      canvas.width = Math.max(1, Math.round(rect.width * pixelRatio));
      canvas.height = Math.max(1, Math.round(rect.height * pixelRatio));
      const context = canvas.getContext("2d");
      if (!context) return;
      context.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0);
      context.fillStyle = "#25313b";
      context.fillRect(0, 0, rect.width, rect.height);

      if (!snapshot || !snapshot.width || !snapshot.height) {
        context.fillStyle = "#91a1ad";
        context.font = "600 13px system-ui";
        context.textAlign = "center";
        context.fillText("等待 SLAM 发布第一帧地图…", rect.width / 2, rect.height / 2);
        return;
      }

      const decoded = decodeRuns(snapshot);
      const raster = document.createElement("canvas");
      raster.width = snapshot.width;
      raster.height = snapshot.height;
      const rasterContext = raster.getContext("2d");
      if (!rasterContext) return;
      const image = rasterContext.createImageData(snapshot.width, snapshot.height);
      for (let sourceY = 0; sourceY < snapshot.height; sourceY += 1) {
        const destinationY = snapshot.height - 1 - sourceY;
        for (let x = 0; x < snapshot.width; x += 1) {
          const value = decoded[sourceY * snapshot.width + x];
          const offset = (destinationY * snapshot.width + x) * 4;
          let color: [number, number, number];
          if (value < 0) color = [43, 56, 66];
          else if (value >= 65) color = [13, 24, 31];
          else if (value === 0) color = [237, 243, 239];
          else {
            const shade = Math.max(80, 230 - value * 2);
            color = [shade, shade + 2, shade];
          }
          image.data[offset] = color[0];
          image.data[offset + 1] = color[1];
          image.data[offset + 2] = color[2];
          image.data[offset + 3] = 255;
        }
      }
      rasterContext.putImageData(image, 0, 0);

      const padding = 34;
      const scale = Math.min(
        (rect.width - padding * 2) / snapshot.width,
        (rect.height - padding * 2) / snapshot.height,
      );
      const drawWidth = snapshot.width * scale;
      const drawHeight = snapshot.height * scale;
      const offsetX = (rect.width - drawWidth) / 2;
      const offsetY = (rect.height - drawHeight) / 2;
      context.imageSmoothingEnabled = false;
      context.shadowColor = "rgba(0,0,0,.36)";
      context.shadowBlur = 24;
      context.drawImage(raster, offsetX, offsetY, drawWidth, drawHeight);
      context.shadowBlur = 0;

      const mapX = (snapshot.robot.x - snapshot.origin.x) / snapshot.resolution;
      const mapY = snapshot.height -
        (snapshot.robot.y - snapshot.origin.y) / snapshot.resolution;
      const robotX = offsetX + mapX * scale;
      const robotY = offsetY + mapY * scale;
      const heading = -snapshot.robot.yaw;
      context.save();
      context.translate(robotX, robotY);
      context.rotate(heading);
      context.shadowColor = "#0aa5ff";
      context.shadowBlur = 16;
      context.fillStyle = "#1b8df5";
      context.beginPath();
      context.moveTo(13, 0);
      context.lineTo(-9, -8);
      context.lineTo(-5, 0);
      context.lineTo(-9, 8);
      context.closePath();
      context.fill();
      context.shadowBlur = 0;
      context.strokeStyle = "#dff5ff";
      context.lineWidth = 2;
      context.stroke();
      context.restore();

      context.fillStyle = "#b8c6cf";
      context.font = "500 10px system-ui";
      context.textAlign = "left";
      context.fillText(
        `${snapshot.width} × ${snapshot.height} · ${snapshot.resolution.toFixed(2)} m/格`,
        offsetX,
        Math.max(18, offsetY - 10),
      );
    };

    draw();
    let resizeFrame = 0;
    const handleResize = () => {
      window.cancelAnimationFrame(resizeFrame);
      resizeFrame = window.requestAnimationFrame(draw);
    };
    window.addEventListener("resize", handleResize);
    return () => {
      window.removeEventListener("resize", handleResize);
      window.cancelAnimationFrame(resizeFrame);
    };
  }, [canvasRef, snapshot]);

  return (
    <div className="live-map-canvas-host" ref={hostRef}>
      <canvas ref={canvasRef} aria-label="实时二维 SLAM 栅格地图" />
    </div>
  );
}

function ManualDrivePad({ disabled, onDrive }: ManualDrivePadProps) {
  const repeatTimer = useRef<number | null>(null);
  const [activeDirection, setActiveDirection] = useState("");

  const stop = useCallback(() => {
    if (repeatTimer.current !== null) {
      window.clearInterval(repeatTimer.current);
      repeatTimer.current = null;
    }
    setActiveDirection("");
    onDrive(0, 0);
  }, [onDrive]);

  useEffect(() => () => {
    if (repeatTimer.current !== null) window.clearInterval(repeatTimer.current);
  }, []);

  const begin = (
    event: ReactPointerEvent<HTMLButtonElement>,
    direction: string,
    linear: number,
    angular: number,
  ) => {
    if (disabled) return;
    event.currentTarget.setPointerCapture(event.pointerId);
    if (repeatTimer.current !== null) window.clearInterval(repeatTimer.current);
    setActiveDirection(direction);
    onDrive(linear, angular);
    repeatTimer.current = window.setInterval(
      () => onDrive(linear, angular),
      120,
    );
  };

  const release = (event: ReactPointerEvent<HTMLButtonElement>) => {
    if (event.currentTarget.hasPointerCapture(event.pointerId)) {
      event.currentTarget.releasePointerCapture(event.pointerId);
    }
    stop();
  };

  return (
    <div className="mapping-drive-pad" aria-label="人工方向控制">
      <button
        type="button"
        disabled={disabled}
        className={`mapping-drive-forward${activeDirection === "forward" ? " active" : ""}`}
        onPointerDown={(event) => begin(event, "forward", 0.24, 0)}
        onPointerUp={release}
        onPointerCancel={release}
        onContextMenu={(event: ReactMouseEvent<HTMLButtonElement>) => event.preventDefault()}
      >
        <b>↑</b><span>前进</span>
      </button>
      <button
        type="button"
        disabled={disabled}
        className={`mapping-drive-left${activeDirection === "left" ? " active" : ""}`}
        onPointerDown={(event) => begin(event, "left", 0, 0.6)}
        onPointerUp={release}
        onPointerCancel={release}
        onContextMenu={(event: ReactMouseEvent<HTMLButtonElement>) => event.preventDefault()}
      >
        <b>↶</b><span>左转</span>
      </button>
      <button
        type="button"
        className="mapping-drive-stop"
        disabled={disabled}
        aria-label="停车"
        onClick={stop}
      >
        <i />
      </button>
      <button
        type="button"
        disabled={disabled}
        className={`mapping-drive-right${activeDirection === "right" ? " active" : ""}`}
        onPointerDown={(event) => begin(event, "right", 0, -0.6)}
        onPointerUp={release}
        onPointerCancel={release}
        onContextMenu={(event: ReactMouseEvent<HTMLButtonElement>) => event.preventDefault()}
      >
        <b>↷</b><span>右转</span>
      </button>
      <button
        type="button"
        disabled={disabled}
        className={`mapping-drive-reverse${activeDirection === "reverse" ? " active" : ""}`}
        onPointerDown={(event) => begin(event, "reverse", -0.24, 0)}
        onPointerUp={release}
        onPointerCancel={release}
        onContextMenu={(event: ReactMouseEvent<HTMLButtonElement>) => event.preventDefault()}
      >
        <b>↓</b><span>后退</span>
      </button>
    </div>
  );
}

export function LiveMappingWorkspace({
  apiBase,
  connected,
  onGatewayChange,
  onSavedMapsChange,
  onEditSavedMap,
  operation,
  onReturnToNavigation,
}: Props) {
  const mapCanvasRef = useRef<HTMLCanvasElement | null>(null);
  const voxelCanvasRef = useRef<HTMLCanvasElement | null>(null);
  const [snapshot, setSnapshot] = useState<LiveMapSnapshot | null>(null);
  const [voxelSnapshot, setVoxelSnapshot] =
    useState<LiveVoxelSnapshot | null>(null);
  const [voxelNotice, setVoxelNotice] = useState(
    "正在等待真实 OctoMap 体素数据",
  );
  const [mapping, setMapping] = useState<MappingRuntime>(EMPTY_MAPPING);
  const [maps, setMaps] = useState<SavedMap[]>([]);
  const [view, setView] = useState<"2d" | "3d">("2d");
  const [notice, setNotice] = useState("等待车辆网关连接");
  const [modalOpen, setModalOpen] = useState(false);
  const [mapName, setMapName] = useState("");
  const [cameraVersion, setCameraVersion] = useState(0);
  const [cameraError, setCameraError] = useState(false);
  const [gatewayDraft, setGatewayDraft] = useState(apiBase);
  const [busyMapId, setBusyMapId] = useState<string | null>(null);
  const [reset3d, setReset3d] = useState(0);
  const driveRef = useRef({ linear: 0, angular: 0 });
  const lastControlAt = useRef(0);
  const controlTimer = useRef<number | null>(null);
  const lastSavedId = useRef<string | null>(null);

  const post = useCallback(async (path: string, payload: object = {}) => {
    if (!apiBase) return false;
    try {
      const response = await fetch(`${apiBase}${path}`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
      });
      if (!response.ok) throw new Error("request failed");
      return true;
    } catch {
      setNotice("无法连接车辆网关，请确认 Ubuntu 仿真和端口 8765");
      return false;
    }
  }, [apiBase]);

  const refreshLibrary = useCallback(async () => {
    if (!apiBase) return;
    try {
      const response = await fetch(`${apiBase}/api/mapping/maps`, {
        cache: "no-store",
      });
      if (!response.ok) return;
      const payload = await response.json() as { maps?: SavedMap[] };
      const nextMaps = Array.isArray(payload.maps) ? payload.maps : [];
      setMaps(nextMaps);
      onSavedMapsChange?.(
        nextMaps
          .map((map) => map.editor_map)
          .filter((map): map is PatrolMap => Boolean(map)),
      );
    } catch {
      // The live map poll owns the connection message.
    }
  }, [apiBase, onSavedMapsChange]);

  useEffect(() => {
    if (!apiBase) return;
    let active = true;
    const poll = async () => {
      try {
        const response = await fetch(`${apiBase}/api/mapping/map`, {
          cache: "no-store",
        });
        if (!response.ok) throw new Error("map unavailable");
        const next = await response.json() as LiveMapSnapshot;
        if (!active) return;
        if (!next.ok) {
          setSnapshot(null);
          setMapping(next.mapping ?? EMPTY_MAPPING);
          setNotice(
            next.mapping?.state === "ERROR"
              ? next.mapping.save_error || next.mapping.detail
              : next.mapping?.detail || "等待开始新的建图任务",
          );
          return;
        }
        setSnapshot(next);
        setMapping(next.mapping ?? EMPTY_MAPPING);
        setNotice(
          next.mapping?.state === "ERROR"
            ? next.mapping.save_error || next.mapping.detail
            : next.mapping?.detail || "实时地图已连接",
        );
      } catch {
        if (active) setNotice("正在等待 SLAM 地图数据…");
      }
    };
    void poll();
    const initialLibraryTimer = window.setTimeout(
      () => void refreshLibrary(),
      0,
    );
    const timer = window.setInterval(poll, 700);
    const libraryTimer = window.setInterval(refreshLibrary, 6000);
    return () => {
      active = false;
      window.clearTimeout(initialLibraryTimer);
      window.clearInterval(timer);
      window.clearInterval(libraryTimer);
    };
  }, [apiBase, refreshLibrary]);

  useEffect(() => {
    if (!apiBase || view !== "3d") return;
    let active = true;
    const pollVoxels = async () => {
      try {
        const response = await fetch(`${apiBase}/api/mapping/voxels`, {
          cache: "no-store",
        });
        if (!response.ok) throw new Error("voxels unavailable");
        const next = await response.json() as LiveVoxelSnapshot;
        if (!active) return;
        setVoxelSnapshot(next);
        setVoxelNotice(
          next.ok
            ? `实时 OctoMap · ${next.source_voxel_count.toLocaleString()} 个占用体素${next.truncated ? "（网页已抽样）" : ""}`
            : next.message || "正在等待真实 OctoMap 体素数据",
        );
      } catch {
        if (!active) return;
        setVoxelSnapshot(null);
        setVoxelNotice("未连接 OctoMap 实时接口，请启动纯 SLAM 三维仿真");
      }
    };
    void pollVoxels();
    const timer = window.setInterval(pollVoxels, 1000);
    return () => {
      active = false;
      window.clearInterval(timer);
    };
  }, [apiBase, view]);

  useEffect(() => {
    const saved = mapping.saved_map;
    if (!saved || lastSavedId.current === saved.id) return;
    lastSavedId.current = saved.id;
    setModalOpen(false);
    setNotice(`“${saved.name}”已保存到地图仓库`);
    void refreshLibrary();
  }, [mapping.saved_map, refreshLibrary]);

  useEffect(() => () => {
    if (controlTimer.current !== null) window.clearTimeout(controlTimer.current);
    void post("/api/control/manual", { linear: 0, angular: 0 });
  }, [post]);

  const transmitControl = useCallback((immediate = false) => {
    const sendNow = () => {
      lastControlAt.current = performance.now();
      controlTimer.current = null;
      void post("/api/control/manual", driveRef.current);
    };
    const remaining = 90 - (performance.now() - lastControlAt.current);
    if (immediate || remaining <= 0) {
      if (controlTimer.current !== null) window.clearTimeout(controlTimer.current);
      sendNow();
    } else if (controlTimer.current === null) {
      controlTimer.current = window.setTimeout(sendNow, remaining);
    }
  }, [post]);

  const setDrive = useCallback((linear: number, angular: number) => {
    driveRef.current = {
      linear: Number(linear.toFixed(3)),
      angular: Number(angular.toFixed(3)),
    };
    transmitControl(true);
  }, [transmitControl]);

  const startManualMapping = async () => {
    if (await post("/api/mapping/start")) {
      setReset3d((value) => value + 1);
      setNotice("手动建图已开始，按住方向按钮驾驶扫图");
    }
  };

  const startExploration = async () => {
    driveRef.current = { linear: 0, angular: 0 };
    if (await post("/api/mapping/explore")) {
      setReset3d((value) => value + 1);
      setNotice("自主探索命令已发送，正在选择首个未知边界");
    }
  };

  const pauseMapping = async () => {
    driveRef.current = { linear: 0, angular: 0 };
    await post("/api/mapping/stop");
    setNotice("停车命令已发送，地图保留在内存中");
  };

  const saveMap = async () => {
    const name = mapName.trim();
    if (!name) return;
    if (await post("/api/mapping/finish", { name })) {
      setNotice("正在保存地图，请勿关闭仿真…");
    }
  };

  const discardMapping = async () => {
    driveRef.current = { linear: 0, angular: 0 };
    if (await post("/api/mapping/discard")) {
      setSnapshot(null);
      setMapping({
        ...EMPTY_MAPPING,
        state: "RESETTING",
        detail: "正在清空地图并让车辆返回原点",
      });
      setView("2d");
      setReset3d((value) => value + 1);
      setNotice("正在放弃本次会话：地图清零，车辆返回原点");
    }
  };

  const deleteMap = async (map: SavedMap) => {
    setBusyMapId(map.id);
    if (await post("/api/mapping/delete", { id: map.id })) {
      const remaining = maps.filter((item) => item.id !== map.id);
      setMaps(remaining);
      onSavedMapsChange?.(
        remaining
          .map((item) => item.editor_map)
          .filter((item): item is PatrolMap => Boolean(item)),
      );
      setNotice(`“${map.name}”已删除`);
    }
    setBusyMapId(null);
  };

  const deployMap = async (map: SavedMap) => {
    setBusyMapId(map.id);
    if (await post("/api/mapping/deploy", { id: map.id })) {
      setNotice(`正在部署“${map.name}”；导航模式下将直接加载到机器人内存`);
    }
    setBusyMapId(null);
  };

  const activeMapping = ["MAPPING", "STARTING", "EXPLORING", "NAVIGATING"]
    .includes(mapping.state);
  const autonomousActive = activeMapping && mapping.mode === "autonomous";
  const blockedByOtherTask = operation.locked && operation.owner !== "mapping";
  const stateCopy = STATE_COPY[mapping.state] ?? {
    label: mapping.state || "等待状态",
    tone: "idle",
  };
  const cameraStream = `${apiBase}/api/camera/stream?v=${cameraVersion}`;
  const knownMegabytes = useMemo(
    () => maps.reduce((total, map) => total + map.size_bytes, 0) / 1024 / 1024,
    [maps],
  );

  return (
    <section className="live-mapping-page">
      <style>{`
        .live-mapping-page{--blue:#1677e8;--cyan:#2fc4ee;--green:#23b989;--red:#da5662;--ink:#173047;min-height:calc(100vh - 74px);padding:18px 20px 24px;background:#f3f6f8;color:var(--ink)}.live-mapping-page *{box-sizing:border-box}.mapping-page-head{display:flex;align-items:flex-end;justify-content:space-between;gap:20px;margin-bottom:14px}.mapping-page-head span{font-size:8px;font-weight:900;letter-spacing:1.8px;color:#2377d1}.mapping-page-head h1{margin:4px 0;font-size:22px}.mapping-page-head p{margin:0;color:#7e8e9e;font-size:10px}.mapping-mode-tabs{display:flex;padding:4px;border:1px solid #dce4eb;border-radius:9px;background:#fff}.mapping-mode-tabs button{height:32px;padding:0 13px;border:0;border-radius:6px;background:transparent;color:#718292;font-size:9px;font-weight:800}.mapping-mode-tabs .active{background:#eaf4ff;color:#176dcc}.mapping-live-grid{display:grid;grid-template-columns:260px minmax(480px,1fr) 285px;gap:12px;height:min(760px,calc(100vh - 145px));min-height:610px}.mapping-column{min-height:0;display:flex;flex-direction:column;gap:12px}.mapping-card{min-height:0;border:1px solid #dce4ea;border-radius:12px;background:#fff;box-shadow:0 10px 28px #173b5510;overflow:hidden}.mapping-card-head{height:49px;padding:0 13px;display:flex;align-items:center;justify-content:space-between;border-bottom:1px solid #e8edf1}.mapping-card-head strong{display:block;font-size:10px}.mapping-card-head span{display:block;margin-top:3px;color:#8997a5;font-size:7px}.mapping-card-head em{font-style:normal;padding:4px 7px;border-radius:10px;background:#edf2f6;color:#718190;font-size:7px;font-weight:850}.mapping-card-head em.online{background:#e2f7ef;color:#16835f}.mapping-camera-card{flex:0 0 210px}.mapping-camera{position:relative;height:161px;background:#07131f;overflow:hidden}.mapping-camera img{width:100%;height:100%;object-fit:cover}.mapping-camera-empty{position:absolute;inset:0;display:grid;place-items:center;text-align:center;color:#7c98aa;font-size:8px;background-image:linear-gradient(#5d8eaa12 1px,transparent 1px),linear-gradient(90deg,#5d8eaa12 1px,transparent 1px);background-size:24px 24px}.mapping-camera-hud{position:absolute;left:9px;right:9px;top:8px;display:flex;justify-content:space-between;color:#d8eaf5;font-size:7px;text-shadow:0 1px 4px #000}.mapping-camera-hud b{color:#6ef0bc}.mapping-camera-retry{position:absolute;right:8px;bottom:8px;border:1px solid #ffffff25;border-radius:5px;background:#091a28cc;color:#d9e7ef;padding:5px 7px;font-size:7px}.mapping-joystick-card{flex:1;display:flex;flex-direction:column}.mapping-sticks{flex:1;display:grid;grid-template-columns:1fr 1fr;align-items:center;gap:8px;padding:13px}.mapping-stick-block{text-align:center}.mapping-stick{position:relative;width:92px;height:92px;margin:auto;border:1px solid #cfdbe4;border-radius:50%;background:radial-gradient(circle,#f9fbfc 0 20%,#ecf2f6 21% 22%,#f8fafb 23% 48%,#dbe5ec 49% 50%,#f7fafb 51%);touch-action:none;box-shadow:inset 0 5px 16px #133b5710}.mapping-stick:before,.mapping-stick:after{content:"";position:absolute;background:#b9c8d2}.mapping-stick:before{left:15px;right:15px;top:50%;height:1px}.mapping-stick:after{top:15px;bottom:15px;left:50%;width:1px}.mapping-stick i{position:absolute;z-index:2;left:50%;top:50%;width:34px;height:34px;margin:-17px;border-radius:50%;background:linear-gradient(145deg,#2696f2,#116bc9);box-shadow:0 5px 12px #155e9d4d,0 0 0 5px #1985e71b;transition:transform .04s}.mapping-stick.disabled{opacity:.45}.mapping-stick-block strong,.mapping-stick-block span{display:block}.mapping-stick-block strong{margin-top:8px;font-size:8px}.mapping-stick-block span{margin-top:3px;color:#8b98a5;font-size:7px}.mapping-manual-note{margin:0 12px 12px;padding:8px;border-radius:7px;background:#f0f5f8;color:#63788b;font-size:7px;line-height:1.5}.mapping-stage{position:relative;background:#25313b}.mapping-stage .mapping-card-head{position:absolute;z-index:4;left:0;right:0;top:0;border-color:#ffffff18;background:#16232eea;color:#edf7fc}.mapping-stage .mapping-card-head span{color:#8fa8b7}.mapping-stage .mapping-card-head em{background:#274051;color:#9fbbcb}.mapping-view-tabs{position:absolute;z-index:5;right:12px;top:61px;display:flex;padding:3px;border:1px solid #ffffff1b;border-radius:7px;background:#101d27d9}.mapping-view-tabs button{height:28px;padding:0 10px;border:0;border-radius:5px;background:transparent;color:#8fa6b4;font-size:7px;font-weight:800}.mapping-view-tabs .active{background:#1a79cc;color:#fff}.live-map-canvas-host{position:absolute;inset:49px 0 0}.live-map-canvas-host canvas{display:block;width:100%;height:100%}.mapping-3d-host{position:absolute;inset:49px 0 0}.mapping-voxel-source{position:absolute;z-index:4;left:12px;top:12px;padding:7px 9px;border:1px solid #ffffff1c;border-radius:7px;background:#0e1d28d9;color:#9fc3d8;font-size:7px;pointer-events:none}.mapping-stage-hud{position:absolute;z-index:5;left:13px;right:13px;bottom:12px;display:flex;align-items:flex-end;justify-content:space-between;pointer-events:none}.mapping-stage-metrics{display:flex;gap:5px}.mapping-stage-metrics span{min-width:82px;padding:7px 9px;border:1px solid #ffffff1a;border-radius:7px;background:#13222dd9;color:#8fa6b5;font-size:7px}.mapping-stage-metrics b{display:block;margin-top:3px;color:#f0f8fc;font-size:10px}.mapping-legend{display:flex;gap:10px;padding:7px 9px;border:1px solid #ffffff1a;border-radius:7px;background:#13222dd9;color:#afc0ca;font-size:7px}.mapping-legend i{display:inline-block;width:7px;height:7px;margin-right:4px;border-radius:1px}.mapping-legend .free{background:#edf3ef}.mapping-legend .occupied{background:#0d181f}.mapping-legend .unknown{background:#2b3842}.mapping-control-card{flex:0 0 auto}.mapping-state{margin:12px;padding:11px;border:1px solid #dfe7ed;border-radius:9px;background:#f7fafb}.mapping-state.active{border-color:#9bdaca;background:#effaf6}.mapping-state.saving{border-color:#efd298;background:#fff9ea}.mapping-state.saved{border-color:#a6decf;background:#edf9f5}.mapping-state.error{border-color:#efb2b7;background:#fff1f2}.mapping-state-top{display:flex;align-items:center;justify-content:space-between}.mapping-state-top strong{font-size:10px}.mapping-state-top strong:before{content:"";display:inline-block;width:7px;height:7px;margin-right:7px;border-radius:50%;background:#91a2af}.mapping-state.active .mapping-state-top strong:before{background:#20bd8b;box-shadow:0 0 0 4px #20bd8b18}.mapping-state.saving .mapping-state-top strong:before{background:#dda33d}.mapping-state.saved .mapping-state-top strong:before{background:#20ad80}.mapping-state.error .mapping-state-top strong:before{background:#d74f5a}.mapping-state-top b{font-size:12px;font-variant-numeric:tabular-nums}.mapping-state p{margin:8px 0 0;color:#6d8192;font-size:7px;line-height:1.5}.mapping-progress{height:4px;margin-top:9px;border-radius:4px;background:#dde6eb;overflow:hidden}.mapping-progress i{display:block;height:100%;background:linear-gradient(90deg,#1683eb,#2ac7a2);transition:width .35s}.mapping-stats{display:grid;grid-template-columns:repeat(2,1fr);gap:6px;margin:0 12px 10px}.mapping-stats div{padding:8px;border-radius:7px;background:#f3f6f8}.mapping-stats small,.mapping-stats b{display:block}.mapping-stats small{color:#8493a1;font-size:7px}.mapping-stats b{margin-top:3px;font-size:10px}.mapping-actions{display:grid;grid-template-columns:1fr 1fr;gap:6px;padding:0 12px 12px}.mapping-actions button{height:34px;border-radius:7px;font-size:7px;font-weight:850}.mapping-actions .primary{border:0;background:#177ce2;color:#fff}.mapping-actions .auto{border:0;background:#21a47d;color:#fff}.mapping-actions .stop{border:1px solid #e5bdc1;background:#fff5f5;color:#bd4c56}.mapping-actions .save{border:1px solid #9edac8;background:#effaf6;color:#117c5d}.mapping-actions .discard{grid-column:1/-1;border:0;background:transparent;color:#a66b70;height:25px}.mapping-actions button:disabled{opacity:.4}.mapping-library-card{flex:1;display:flex;min-height:0;flex-direction:column}.mapping-storage{padding:9px 12px;border-bottom:1px solid #e9edf1;background:#f8fafb;color:#738493;font-size:7px}.mapping-map-list{flex:1;min-height:0;overflow:auto;padding:8px}.mapping-map-row{padding:9px;margin-bottom:7px;border:1px solid #e1e8ed;border-radius:8px}.mapping-map-row strong{display:block;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;font-size:8px}.mapping-map-row>span{display:block;margin-top:4px;color:#82919e;font-size:7px}.mapping-map-meta{display:flex;gap:9px;margin-top:6px;color:#5e7487;font-size:7px}.mapping-map-actions{display:grid;grid-template-columns:1fr 54px;gap:5px;margin-top:7px}.mapping-map-actions button{height:27px;border-radius:5px;font-size:7px;font-weight:800}.mapping-map-actions .deploy{border:0;background:#eaf4ff;color:#176fcf}.mapping-map-actions .delete{border:1px solid #eccbd0;background:#fff;color:#c5515b}.mapping-empty{padding:40px 18px;text-align:center;color:#8292a0;font-size:8px;line-height:1.6}.mapping-notice{margin:8px;padding:8px;border-radius:7px;background:#edf4fa;color:#536f86;font-size:7px;line-height:1.5}.mapping-save-backdrop{position:fixed;z-index:1000;inset:0;display:grid;place-items:center;padding:18px;background:#07121db5;backdrop-filter:blur(5px)}.mapping-save-modal{width:min(480px,100%);border-radius:13px;background:#fff;box-shadow:0 28px 80px #0007;overflow:hidden}.mapping-save-head{display:flex;align-items:center;justify-content:space-between;padding:15px 17px;border-bottom:1px solid #e7ecf0}.mapping-save-head h2{margin:0;font-size:13px}.mapping-save-head p{margin:4px 0 0;color:#82919f;font-size:8px}.mapping-save-head button{width:28px;height:28px;border:0;border-radius:6px;background:#eef2f5;color:#657789}.mapping-save-body{padding:17px}.mapping-save-body label{display:block;color:#65788a;font-size:8px;font-weight:800}.mapping-save-body input{display:block;width:100%;height:38px;margin-top:7px;padding:0 10px;border:1px solid #d9e3ea;border-radius:7px;font-size:10px}.mapping-save-summary{display:grid;grid-template-columns:repeat(3,1fr);gap:7px;margin-top:12px}.mapping-save-summary div{padding:9px;border-radius:7px;background:#f2f6f8}.mapping-save-summary small,.mapping-save-summary b{display:block}.mapping-save-summary small{font-size:7px;color:#82919f}.mapping-save-summary b{margin-top:4px;font-size:9px}.mapping-save-foot{display:flex;justify-content:flex-end;gap:7px;padding:12px 17px;border-top:1px solid #e8edf1;background:#f9fbfc}.mapping-save-foot button{height:33px;padding:0 13px;border-radius:6px;font-size:8px;font-weight:800}.mapping-save-foot .cancel{border:1px solid #dce4ea;background:#fff;color:#657788}.mapping-save-foot .confirm{border:0;background:#187de2;color:#fff}.mapping-save-foot .confirm:disabled{opacity:.4}@media(max-width:1250px){.mapping-live-grid{grid-template-columns:230px minmax(430px,1fr) 285px;gap:12px;height:min(760px,calc(100vh - 145px));min-height:610px}.mapping-column{min-height:0;display:flex;flex-direction:column;gap:12px}.mapping-card{min-height:0;border:1px solid #dce4ea;border-radius:12px;background:#fff;box-shadow:0 10px 28px #173b5510;overflow:hidden}.mapping-card-head{height:49px;padding:0 13px;display:flex;align-items:center;justify-content:space-between;border-bottom:1px solid #e8edf1}.mapping-card-head strong{display:block;font-size:10px}.mapping-card-head span{display:block;margin-top:3px;color:#8997a5;font-size:7px}.mapping-card-head em{font-style:normal;padding:4px 7px;border-radius:10px;background:#edf2f6;color:#718190;font-size:7px;font-weight:850}.mapping-card-head em.online{background:#e2f7ef;color:#16835f}.mapping-camera-card{flex:0 0 210px}.mapping-camera{position:relative;height:161px;background:#07131f;overflow:hidden}.mapping-camera img{width:100%;height:100%;object-fit:cover}.mapping-camera-empty{position:absolute;inset:0;display:grid;place-items:center;text-align:center;color:#7c98aa;font-size:8px;background-image:linear-gradient(#5d8eaa12 1px,transparent 1px),linear-gradient(90deg,#5d8eaa12 1px,transparent 1px);background-size:24px 24px}.mapping-camera-hud{position:absolute;left:9px;right:9px;top:8px;display:flex;justify-content:space-between;color:#d8eaf5;font-size:7px;text-shadow:0 1px 4px #000}.mapping-camera-hud b{color:#6ef0bc}.mapping-camera-retry{position:absolute;right:8px;bottom:8px;border:1px solid #ffffff25;border-radius:5px;background:#091a28cc;color:#d9e7ef;padding:5px 7px;font-size:7px}.mapping-joystick-card{flex:1;display:flex;flex-direction:column}.mapping-sticks{flex:1;display:grid;grid-template-columns:1fr 1fr;align-items:center;gap:8px;padding:13px}.mapping-stick-block{text-align:center}.mapping-stick{position:relative;width:92px;height:92px;margin:auto;border:1px solid #cfdbe4;border-radius:50%;background:radial-gradient(circle,#f9fbfc 0 20%,#ecf2f6 21% 22%,#f8fafb 23% 48%,#dbe5ec 49% 50%,#f7fafb 51%);touch-action:none;box-shadow:inset 0 5px 16px #133b5710}.mapping-stick:before,.mapping-stick:after{content:"";position:absolute;background:#b9c8d2}.mapping-stick:before{left:15px;right:15px;top:50%;height:1px}.mapping-stick:after{top:15px;bottom:15px;left:50%;width:1px}.mapping-stick i{position:absolute;z-index:2;left:50%;top:50%;width:34px;height:34px;margin:-17px;border-radius:50%;background:linear-gradient(145deg,#2696f2,#116bc9);box-shadow:0 5px 12px #155e9d4d,0 0 0 5px #1985e71b;transition:transform .04s}.mapping-stick.disabled{opacity:.45}.mapping-stick-block strong,.mapping-stick-block span{display:block}.mapping-stick-block strong{margin-top:8px;font-size:8px}.mapping-stick-block span{margin-top:3px;color:#8b98a5;font-size:7px}.mapping-manual-note{margin:0 12px 12px;padding:8px;border-radius:7px;background:#f0f5f8;color:#63788b;font-size:7px;line-height:1.5}.mapping-stage{position:relative;background:#25313b}.mapping-stage .mapping-card-head{position:absolute;z-index:4;left:0;right:0;top:0;border-color:#ffffff18;background:#16232eea;color:#edf7fc}.mapping-stage .mapping-card-head span{color:#8fa8b7}.mapping-stage .mapping-card-head em{background:#274051;color:#9fbbcb}.mapping-view-tabs{position:absolute;z-index:5;right:12px;top:61px;display:flex;padding:3px;border:1px solid #ffffff1b;border-radius:7px;background:#101d27d9}.mapping-view-tabs button{height:28px;padding:0 10px;border:0;border-radius:5px;background:transparent;color:#8fa6b4;font-size:7px;font-weight:800}.mapping-view-tabs .active{background:#1a79cc;color:#fff}.live-map-canvas-host{position:absolute;inset:49px 0 0}.live-map-canvas-host canvas{display:block;width:100%;height:100%}.mapping-3d-host{position:absolute;inset:49px 0 0}.mapping-voxel-source{position:absolute;z-index:4;left:12px;top:12px;padding:7px 9px;border:1px solid #ffffff1c;border-radius:7px;background:#0e1d28d9;color:#9fc3d8;font-size:7px;pointer-events:none}.mapping-stage-hud{position:absolute;z-index:5;left:13px;right:13px;bottom:12px;display:flex;align-items:flex-end;justify-content:space-between;pointer-events:none}.mapping-stage-metrics{display:flex;gap:5px}.mapping-stage-metrics span{min-width:82px;padding:7px 9px;border:1px solid #ffffff1a;border-radius:7px;background:#13222dd9;color:#8fa6b5;font-size:7px}.mapping-stage-metrics b{display:block;margin-top:3px;color:#f0f8fc;font-size:10px}.mapping-legend{display:flex;gap:10px;padding:7px 9px;border:1px solid #ffffff1a;border-radius:7px;background:#13222dd9;color:#afc0ca;font-size:7px}.mapping-legend i{display:inline-block;width:7px;height:7px;margin-right:4px;border-radius:1px}.mapping-legend .free{background:#edf3ef}.mapping-legend .occupied{background:#0d181f}.mapping-legend .unknown{background:#2b3842}.mapping-control-card{flex:0 0 auto}.mapping-state{margin:12px;padding:11px;border:1px solid #dfe7ed;border-radius:9px;background:#f7fafb}.mapping-state.active{border-color:#9bdaca;background:#effaf6}.mapping-state.saving{border-color:#efd298;background:#fff9ea}.mapping-state.saved{border-color:#a6decf;background:#edf9f5}.mapping-state.error{border-color:#efb2b7;background:#fff1f2}.mapping-state-top{display:flex;align-items:center;justify-content:space-between}.mapping-state-top strong{font-size:10px}.mapping-state-top strong:before{content:"";display:inline-block;width:7px;height:7px;margin-right:7px;border-radius:50%;background:#91a2af}.mapping-state.active .mapping-state-top strong:before{background:#20bd8b;box-shadow:0 0 0 4px #20bd8b18}.mapping-state.saving .mapping-state-top strong:before{background:#dda33d}.mapping-state.saved .mapping-state-top strong:before{background:#20ad80}.mapping-state.error .mapping-state-top strong:before{background:#d74f5a}.mapping-state-top b{font-size:12px;font-variant-numeric:tabular-nums}.mapping-state p{margin:8px 0 0;color:#6d8192;font-size:7px;line-height:1.5}.mapping-progress{height:4px;margin-top:9px;border-radius:4px;background:#dde6eb;overflow:hidden}.mapping-progress i{display:block;height:100%;background:linear-gradient(90deg,#1683eb,#2ac7a2);transition:width .35s}.mapping-stats{display:grid;grid-template-columns:repeat(2,1fr);gap:6px;margin:0 12px 10px}.mapping-stats div{padding:8px;border-radius:7px;background:#f3f6f8}.mapping-stats small,.mapping-stats b{display:block}.mapping-stats small{color:#8493a1;font-size:7px}.mapping-stats b{margin-top:3px;font-size:10px}.mapping-actions{display:grid;grid-template-columns:1fr 1fr;gap:6px;padding:0 12px 12px}.mapping-actions button{height:34px;border-radius:7px;font-size:7px;font-weight:850}.mapping-actions .primary{border:0;background:#177ce2;color:#fff}.mapping-actions .auto{border:0;background:#21a47d;color:#fff}.mapping-actions .stop{border:1px solid #e5bdc1;background:#fff5f5;color:#bd4c56}.mapping-actions .save{border:1px solid #9edac8;background:#effaf6;color:#117c5d}.mapping-actions .discard{grid-column:1/-1;border:0;background:transparent;color:#a66b70;height:25px}.mapping-actions button:disabled{opacity:.4}.mapping-library-card{flex:1;display:flex;min-height:0;flex-direction:column}.mapping-storage{padding:9px 12px;border-bottom:1px solid #e9edf1;background:#f8fafb;color:#738493;font-size:7px}.mapping-map-list{flex:1;min-height:0;overflow:auto;padding:8px}.mapping-map-row{padding:9px;margin-bottom:7px;border:1px solid #e1e8ed;border-radius:8px}.mapping-map-row strong{display:block;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;font-size:8px}.mapping-map-row>span{display:block;margin-top:4px;color:#82919e;font-size:7px}.mapping-map-meta{display:flex;gap:9px;margin-top:6px;color:#5e7487;font-size:7px}.mapping-map-actions{display:grid;grid-template-columns:1fr 54px;gap:5px;margin-top:7px}.mapping-map-actions button{height:27px;border-radius:5px;font-size:7px;font-weight:800}.mapping-map-actions .deploy{border:0;background:#eaf4ff;color:#176fcf}.mapping-map-actions .delete{border:1px solid #eccbd0;background:#fff;color:#c5515b}.mapping-empty{padding:40px 18px;text-align:center;color:#8292a0;font-size:8px;line-height:1.6}.mapping-notice{margin:8px;padding:8px;border-radius:7px;background:#edf4fa;color:#536f86;font-size:7px;line-height:1.5}.mapping-save-backdrop{position:fixed;z-index:1000;inset:0;display:grid;place-items:center;padding:18px;background:#07121db5;backdrop-filter:blur(5px)}.mapping-save-modal{width:min(480px,100%);border-radius:13px;background:#fff;box-shadow:0 28px 80px #0007;overflow:hidden}.mapping-save-head{display:flex;align-items:center;justify-content:space-between;padding:15px 17px;border-bottom:1px solid #e7ecf0}.mapping-save-head h2{margin:0;font-size:13px}.mapping-save-head p{margin:4px 0 0;color:#82919f;font-size:8px}.mapping-save-head button{width:28px;height:28px;border:0;border-radius:6px;background:#eef2f5;color:#657789}.mapping-save-body{padding:17px}.mapping-save-body label{display:block;color:#65788a;font-size:8px;font-weight:800}.mapping-save-body input{display:block;width:100%;height:38px;margin-top:7px;padding:0 10px;border:1px solid #d9e3ea;border-radius:7px;font-size:10px}.mapping-save-summary{display:grid;grid-template-columns:repeat(3,1fr);gap:7px;margin-top:12px}.mapping-save-summary div{padding:9px;border-radius:7px;background:#f2f6f8}.mapping-save-summary small,.mapping-save-summary b{display:block}.mapping-save-summary small{font-size:7px;color:#82919f}.mapping-save-summary b{margin-top:4px;font-size:9px}.mapping-save-foot{display:flex;justify-content:flex-end;gap:7px;padding:12px 17px;border-top:1px solid #e8edf1;background:#f9fbfc}.mapping-save-foot button{height:33px;padding:0 13px;border-radius:6px;font-size:8px;font-weight:800}.mapping-save-foot .cancel{border:1px solid #dce4ea;background:#fff;color:#657788}.mapping-save-foot .confirm{border:0;background:#187de2;color:#fff}.mapping-save-foot .confirm:disabled{opacity:.4}@media(max-width:1250px){.mapping-live-grid{grid-template-columns:230px minmax(430px,1fr);height:auto}.mapping-column.right{grid-column:1/-1}.mapping-library-card{height:300px}.mapping-map-list{display:grid;grid-template-columns:repeat(3,1fr);gap:7px}.mapping-map-row{margin:0}}@media(max-width:820px){.live-mapping-page{padding:12px}.mapping-page-head{display:block}.mapping-mode-tabs{width:max-content;margin-top:11px}.mapping-live-grid{display:block}.mapping-column,.mapping-stage{margin-bottom:10px}.mapping-camera-card{height:220px}.mapping-joystick-card{height:260px}.mapping-stage{height:520px}.mapping-control-card{margin-bottom:10px}.mapping-map-list{grid-template-columns:1fr 1fr}}@media(max-width:520px){.mapping-map-list{grid-template-columns:1fr}.mapping-stage-metrics span:nth-child(3){display:none}.mapping-legend{display:none}}
        .mapping-connection{display:flex;align-items:center;gap:10px;margin:-2px 0 13px;padding:10px 12px;border:1px solid #efc477;border-radius:9px;background:#fff8e9;color:#72572a}.mapping-connection strong,.mapping-connection span{white-space:nowrap}.mapping-connection strong{font-size:9px}.mapping-connection span{font-size:8px}.mapping-connection input{min-width:220px;flex:1;height:31px;padding:0 9px;border:1px solid #e2c992;border-radius:6px;background:#fff;color:#35495c;font-size:9px;outline:0}.mapping-connection input:focus{border-color:#4e99df;box-shadow:0 0 0 3px #2783d916}.mapping-connection button{height:31px;padding:0 13px;border:0;border-radius:6px;background:#177ce2;color:#fff;font-size:8px;font-weight:850}@media(max-width:700px){.mapping-connection{flex-wrap:wrap}.mapping-connection span{display:none}.mapping-connection input{min-width:170px}}
        .mapping-drive-pad{flex:1;display:grid;grid-template-columns:repeat(3,70px);grid-template-rows:70px 70px 70px;justify-content:center;align-content:center;gap:8px;padding:16px 10px 10px;touch-action:none}.mapping-drive-pad button{display:flex;flex-direction:column;align-items:center;justify-content:center;border:1px solid #d2deea;border-radius:13px;background:#f8fbff;color:#1760a7;box-shadow:0 5px 14px #22466412;touch-action:none;user-select:none}.mapping-drive-pad button b{font-size:25px;font-weight:400;line-height:1}.mapping-drive-pad button span{margin-top:3px;font-size:9px;font-weight:850}.mapping-drive-pad button.active{border-color:#72ace7;background:#e5f2ff;transform:translateY(1px);box-shadow:inset 0 2px 7px #267ac426}.mapping-drive-pad button:disabled{opacity:.38}.mapping-drive-pad button:nth-child(1){grid-column:2;grid-row:1}.mapping-drive-pad button:nth-child(2){grid-column:1;grid-row:2}.mapping-drive-pad button:nth-child(3){grid-column:2;grid-row:2}.mapping-drive-pad button:nth-child(4){grid-column:3;grid-row:2}.mapping-drive-pad button:nth-child(5){grid-column:2;grid-row:3}.mapping-drive-pad .mapping-drive-stop{border-color:#ffb8bc;background:#fff3f3}.mapping-drive-stop i{width:13px;height:13px;background:#df4d59}.mapping-3d-host .slam-viewer-canvas-wrap{position:absolute;inset:0;min-width:0;min-height:0;overflow:hidden}.mapping-3d-host canvas{display:block;width:100%;height:100%}.mapping-map-actions{grid-template-columns:1fr 1fr 48px}.mapping-map-actions .edit{border:0;background:#e9f8f3;color:#168063}.mapping-map-format{display:inline-flex!important;margin-top:6px!important;padding:3px 6px;border-radius:8px;background:#edf8f5;color:#19785f!important;font-weight:800}
      `}</style>

      <header className="mapping-page-head">
        <div>
          <span>MAPPING MODE · LIVE ROS</span>
          <h1>一键远程建图</h1>
          <p>相机监控、网页遥控、前沿探索与实时二维栅格地图</p>
        </div>
        <div className="mapping-mode-tabs" role="tablist">
          <button type="button" onClick={onReturnToNavigation}>日常巡检导航</button>
          <button type="button" className="active">自主建图模式</button>
        </div>
      </header>

      {!connected && (
        <form
          className="mapping-connection"
          onSubmit={(event) => {
            event.preventDefault();
            const raw = gatewayDraft.trim();
            if (!raw) return;
            const normalized = (
              /^https?:\/\//i.test(raw) ? raw : `http://${raw}`
            ).replace(/\/$/, "");
            setGatewayDraft(normalized);
            setNotice("正在连接车辆网关…");
            onGatewayChange(normalized);
          }}
        >
          <strong>车辆网关未连接</strong>
          <span>请输入 Ubuntu 地址</span>
          <input
            value={gatewayDraft}
            placeholder="http://172.16.194.128:8765"
            aria-label="车辆网关地址"
            onChange={(event) => setGatewayDraft(event.target.value)}
          />
          <button type="submit">连接车辆</button>
        </form>
      )}

      <div className="mapping-live-grid">
        <aside className="mapping-column">
          <section className="mapping-card mapping-camera-card">
            <div className="mapping-card-head">
              <div><strong>相机实时画面</strong><span>RGB-D · MJPEG</span></div>
              <em className={snapshot?.camera.ok ? "online" : ""}>
                {snapshot?.camera.ok ? `${snapshot.camera.fps || 0} FPS` : "等待视频"}
              </em>
            </div>
            <div className="mapping-camera">
              {!cameraError && snapshot?.camera.enabled && (
                <img
                  src={cameraStream}
                  alt="机器人相机实时画面"
                  onError={() => setCameraError(true)}
                  onLoad={() => setCameraError(false)}
                />
              )}
              {(cameraError || !snapshot?.camera.enabled) && (
                <div className="mapping-camera-empty">
                  <span>正在等待机器人相机画面<br />地图数据不受影响</span>
                </div>
              )}
              <div className="mapping-camera-hud">
                <span><b>● LIVE</b>　巡检车 · 01</span>
                <span>FRAME {snapshot?.camera.frames ?? 0}</span>
              </div>
              <button
                type="button"
                className="mapping-camera-retry"
                onClick={() => {
                  setCameraError(false);
                  setCameraVersion((value) => value + 1);
                  void post("/api/camera/enable", { enabled: true });
                }}
              >
                重连画面
              </button>
            </div>
          </section>

          <section className="mapping-card mapping-joystick-card">
            <div className="mapping-card-head">
              <div><strong>人工控制</strong><span>按住方向键移动 · 松手立即停车</span></div>
              <em>{activeMapping ? "可接管" : "待机"}</em>
            </div>
            <ManualDrivePad
              disabled={!connected || !activeMapping || blockedByOtherTask}
              onDrive={setDrive}
            />
            <p className="mapping-manual-note">
              按住方向按钮会自动暂停自主探索并接管底盘；松手后立即发送零速度。
            </p>
          </section>
        </aside>

        <main className="mapping-card mapping-stage">
          <div className="mapping-card-head">
            <div><strong>动态地图画布</strong><span>未知区域随着机器人移动逐步展开</span></div>
            <em className={connected ? "online" : ""}>
              {connected ? "ROS LIVE" : "OFFLINE"}
            </em>
          </div>
          <div className="mapping-view-tabs">
            <button type="button" className={view === "2d" ? "active" : ""} onClick={() => setView("2d")}>2D 栅格</button>
            <button type="button" className={view === "3d" ? "active" : ""} onClick={() => setView("3d")}>3D 体素</button>
          </div>
          {view === "2d" ? (
            <LiveOccupancyCanvas snapshot={snapshot} canvasRef={mapCanvasRef} />
          ) : (
            <div className="mapping-3d-host">
              <div className="mapping-voxel-source">{voxelNotice}</div>
              <VoxelMapScene
                active={activeMapping}
                resetSignal={reset3d}
                showFov
                showLidar
                canvasRef={voxelCanvasRef}
                onMetrics={() => undefined}
                live
                voxelSnapshot={voxelSnapshot}
              />
            </div>
          )}
          <div className="mapping-stage-hud">
            <div className="mapping-stage-metrics">
              <span>探索覆盖<b>{(snapshot?.coverage ?? 0).toFixed(1)}%</b></span>
              <span>已知栅格<b>{(snapshot?.known_cells ?? 0).toLocaleString()}</b></span>
              <span>机器人坐标<b>{snapshot ? `${snapshot.robot.x.toFixed(1)}, ${snapshot.robot.y.toFixed(1)}` : "--"}</b></span>
            </div>
            {view === "2d" && (
              <div className="mapping-legend">
                <span><i className="free" />自由区</span>
                <span><i className="occupied" />障碍</span>
                <span><i className="unknown" />未知</span>
              </div>
            )}
          </div>
        </main>

        <aside className="mapping-column right">
          <section className="mapping-card mapping-control-card">
            <div className="mapping-card-head">
              <div><strong>建图控制台</strong><span>SLAM + Frontier Explorer</span></div>
              <em className={connected ? "online" : ""}>{connected ? "网关在线" : "未连接"}</em>
            </div>
            <div className={`mapping-state ${stateCopy.tone}`}>
              <div className="mapping-state-top">
                <strong>{stateCopy.label}</strong>
                <b>{formatDuration(mapping.duration_seconds)}</b>
              </div>
              <p>
                {mapping.state === "ERROR"
                  ? mapping.save_error || mapping.detail
                  : mapping.detail}
              </p>
              <div className="mapping-progress"><i style={{ width: `${Math.min(100, mapping.coverage || 0)}%` }} /></div>
            </div>
            <div className="mapping-stats">
              <div><small>到达前沿</small><b>{mapping.goals_reached}</b></div>
              <div><small>当前前沿</small><b>{mapping.frontier_clusters}</b></div>
              <div><small>地图尺寸</small><b>{snapshot ? `${snapshot.width} × ${snapshot.height}` : "--"}</b></div>
              <div><small>分辨率</small><b>{snapshot ? `${Math.round(snapshot.resolution * 100)} cm` : "--"}</b></div>
            </div>
            <div className="mapping-actions">
              <button type="button" className="primary" disabled={!connected || activeMapping || blockedByOtherTask} onClick={startManualMapping}>{operation.owner === "patrol" ? "巡检任务运行中" : operation.owner === "map" ? "地图切换中" : "开始建图"}</button>
              <button type="button" className="auto" disabled={!connected || autonomousActive || blockedByOtherTask || mapping.autonomous_available === false} onClick={startExploration}>{mapping.autonomous_available === false ? "纯 SLAM 调图模式" : operation.owner === "patrol" ? "巡检任务运行中" : operation.owner === "map" ? "地图切换中" : "一键自主探路"}</button>
              <button type="button" className="stop" disabled={!connected || !activeMapping} onClick={pauseMapping}>停车 / 暂停</button>
              <button
                type="button"
                className="save"
                disabled={!connected || mapping.state === "SAVING" || !snapshot}
                onClick={() => {
                  setMapName(`自主地图 ${new Date().toLocaleDateString("zh-CN")}`);
                  setModalOpen(true);
                }}
              >
                停止并保存
              </button>
              <button type="button" className="discard" disabled={!connected || mapping.state === "SAVING"} onClick={discardMapping}>放弃本次会话</button>
            </div>
          </section>

          <section className="mapping-card mapping-library-card">
            <div className="mapping-card-head">
              <div><strong>地图存储仓库</strong><span>导航栅格与 SLAM 元数据</span></div>
              <em>{maps.length} MAPS</em>
            </div>
            <div className="mapping-storage">
              已保存 {maps.length} 张 · 占用 {knownMegabytes.toFixed(2)} MB
            </div>
            <div className="mapping-map-list">
              {maps.length === 0 ? (
                <div className="mapping-empty">尚无已保存地图<br />完成建图后会自动出现在这里</div>
              ) : maps.map((map) => (
                <article className="mapping-map-row" key={map.id}>
                  <strong title={map.name}>{map.name}</strong>
                  <span>{formatDate(map.created_at)} · 覆盖 {map.coverage.toFixed(1)}%</span>
                  <span className="mapping-map-format">
                    {map.has_3d ? "2D 栅格 + 3D OctoMap" : "2D 栅格"}
                  </span>
                  <div className="mapping-map-meta">
                    <span>{(map.size_bytes / 1024 / 1024).toFixed(2)} MB</span>
                    <span>{Math.round(map.resolution * 100)} cm</span>
                    <span>{map.width} × {map.height}</span>
                  </div>
                  <div className="mapping-map-actions">
                    <button type="button" className="deploy" disabled={busyMapId !== null} onClick={() => deployMap(map)}>应用此地图 / 部署</button>
                    <button
                      type="button"
                      className="edit"
                      disabled={!map.editor_map}
                      onClick={() => onEditSavedMap?.(map.id)}
                    >
                      设置巡检点
                    </button>
                    <button type="button" className="delete" disabled={busyMapId !== null} onClick={() => deleteMap(map)}>{busyMapId === map.id ? "…" : "删除"}</button>
                  </div>
                </article>
              ))}
            </div>
            <p className="mapping-notice" role="status">{notice}</p>
          </section>
        </aside>
      </div>

      {modalOpen && (
        <div className="mapping-save-backdrop" role="presentation" onMouseDown={(event) => {
          if (event.target === event.currentTarget) setModalOpen(false);
        }}>
          <div className="mapping-save-modal" role="dialog" aria-modal="true" aria-labelledby="mapping-save-title">
            <div className="mapping-save-head">
              <div>
                <h2 id="mapping-save-title">停止并保存地图</h2>
                <p>车辆会先停车，再将二维栅格图写入地图仓库</p>
              </div>
              <button type="button" onClick={() => setModalOpen(false)} aria-label="关闭">×</button>
            </div>
            <div className="mapping-save-body">
              <label>
                地图名称
                <input
                  autoFocus
                  value={mapName}
                  maxLength={48}
                  placeholder="例如：chemical_plant_a"
                  onChange={(event) => setMapName(event.target.value)}
                />
              </label>
              <div className="mapping-save-summary">
                <div><small>持续时间</small><b>{formatDuration(mapping.duration_seconds)}</b></div>
                <div><small>探索覆盖</small><b>{(snapshot?.coverage ?? 0).toFixed(1)}%</b></div>
                <div><small>保存格式</small><b>YAML + PGM</b></div>
              </div>
            </div>
            <div className="mapping-save-foot">
              <button type="button" className="cancel" onClick={() => setModalOpen(false)}>取消</button>
              <button type="button" className="confirm" disabled={!mapName.trim() || mapping.state === "SAVING"} onClick={saveMap}>确认保存</button>
            </div>
          </div>
        </div>
      )}
    </section>
  );
}

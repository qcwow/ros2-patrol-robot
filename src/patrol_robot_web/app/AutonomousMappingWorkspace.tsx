"use client";

/* eslint-disable @next/next/no-img-element -- previews may be local uploads/data URLs. */

import {
  useCallback,
  useEffect,
  useMemo,
  useRef,
  useState,
  type MutableRefObject,
} from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

type MappingStatus = "idle" | "mapping" | "saving" | "saved";

type MapRecord = {
  id: string;
  name: string;
  createdAt: string;
  sizeMb: number;
  voxelCount: number;
  resolution: number;
  deployed: boolean;
  preview?: string;
};

type ViewerMetrics = {
  voxelCount: number;
  coverage: number;
  robotX: number;
  robotZ: number;
};

export type LiveVoxelSnapshot = {
  ok: boolean;
  revision?: number;
  frame_id?: string;
  encoding?: "base64-f32le-xyzsrgb";
  stride?: number;
  data?: string;
  voxel_count: number;
  source_voxel_count: number;
  truncated?: boolean;
  message?: string;
  robot: { x: number; y: number; yaw: number };
};

type Props = {
  onReturnToNavigation?: () => void;
};

const MOCK_MAPS: MapRecord[] = [
  {
    id: "octomap-pipeline-a",
    name: "一号管廊 · 完整扫描",
    createdAt: "2026-07-21T14:32:00+08:00",
    sizeMb: 18.6,
    voxelCount: 182420,
    resolution: 0.08,
    deployed: true,
  },
  {
    id: "octomap-valve-zone",
    name: "阀门作业区 · 夜间",
    createdAt: "2026-07-18T22:06:00+08:00",
    sizeMb: 12.4,
    voxelCount: 126840,
    resolution: 0.08,
    deployed: false,
  },
  {
    id: "octomap-loading-bay",
    name: "装卸区 B · 初始地图",
    createdAt: "2026-07-12T09:18:00+08:00",
    sizeMb: 8.9,
    voxelCount: 91520,
    resolution: 0.1,
    deployed: false,
  },
];

const STATUS_COPY: Record<MappingStatus, { label: string; detail: string }> = {
  idle: { label: "空闲中", detail: "等待开始新的探索任务" },
  mapping: { label: "探索建图中", detail: "前沿探索、SLAM 与体素融合正在运行" },
  saving: { label: "地图保存中", detail: "正在冻结轨迹并生成地图文件" },
  saved: { label: "已保存", detail: "地图已经加入存储仓库" },
};

const delay = (milliseconds: number) =>
  new Promise<void>((resolve) => window.setTimeout(resolve, milliseconds));

/**
 * Mock API boundary. Replace this function with fetch(apiBase + endpoint)
 * when the ROS Web gateway exposes the matching mapping endpoints.
 */
async function sendMockMappingRequest(
  endpoint: string,
  payload?: Record<string, unknown>,
) {
  await delay(620 + Math.random() * 420);
  console.info(`[mapping mock] POST ${endpoint}`, payload ?? {});
  return { ok: true, requestId: `mock-${Date.now()}` };
}

function formatDuration(seconds: number) {
  const hours = Math.floor(seconds / 3600);
  const minutes = Math.floor((seconds % 3600) / 60);
  const remainder = seconds % 60;
  return [hours, minutes, remainder]
    .map((value) => String(value).padStart(2, "0"))
    .join(":");
}

function formatMapDate(value: string) {
  return new Intl.DateTimeFormat("zh-CN", {
    month: "2-digit",
    day: "2-digit",
    hour: "2-digit",
    minute: "2-digit",
    hour12: false,
  }).format(new Date(value));
}

export function VoxelMapScene({
  active,
  resetSignal,
  showFov,
  showLidar,
  canvasRef,
  onMetrics,
  live = false,
  voxelSnapshot = null,
}: {
  active: boolean;
  resetSignal: number;
  showFov: boolean;
  showLidar: boolean;
  canvasRef: MutableRefObject<HTMLCanvasElement | null>;
  onMetrics: (metrics: ViewerMetrics) => void;
  live?: boolean;
  voxelSnapshot?: LiveVoxelSnapshot | null;
}) {
  const hostRef = useRef<HTMLDivElement>(null);
  const activeRef = useRef(active);
  const showFovRef = useRef(showFov);
  const showLidarRef = useRef(showLidar);
  const onMetricsRef = useRef(onMetrics);
  const voxelSnapshotRef = useRef(voxelSnapshot);

  useEffect(() => {
    activeRef.current = active;
  }, [active]);

  useEffect(() => {
    showFovRef.current = showFov;
  }, [showFov]);

  useEffect(() => {
    showLidarRef.current = showLidar;
  }, [showLidar]);

  useEffect(() => {
    onMetricsRef.current = onMetrics;
  }, [onMetrics]);

  useEffect(() => {
    voxelSnapshotRef.current = voxelSnapshot;
  }, [voxelSnapshot]);

  useEffect(() => {
    const host = hostRef.current;
    const canvas = canvasRef.current;
    if (!host || !canvas) return;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x07111e);
    scene.fog = new THREE.FogExp2(0x07111e, 0.027);

    const camera = new THREE.PerspectiveCamera(48, 1, 0.1, 120);
    camera.position.set(12, 14, 17);

    const renderer = new THREE.WebGLRenderer({
      canvas,
      antialias: true,
      preserveDrawingBuffer: true,
    });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.05;

    const controls = new OrbitControls(camera, canvas);
    controls.enableDamping = true;
    controls.dampingFactor = 0.075;
    controls.minDistance = 5;
    controls.maxDistance = 44;
    controls.maxPolarAngle = Math.PI * 0.49;
    controls.target.set(0, 1.2, 0);
    controls.update();

    scene.add(new THREE.HemisphereLight(0x9edcff, 0x112234, 1.55));
    const keyLight = new THREE.DirectionalLight(0xe5f7ff, 2.2);
    keyLight.position.set(8, 18, 11);
    scene.add(keyLight);
    const fillLight = new THREE.PointLight(0x28b8ff, 26, 24, 2);
    fillLight.position.set(-6, 4, -5);
    scene.add(fillLight);

    const grid = new THREE.GridHelper(36, 72, 0x285575, 0x122b3d);
    const gridMaterial = grid.material as THREE.Material;
    gridMaterial.transparent = true;
    gridMaterial.opacity = 0.38;
    scene.add(grid);

    const scanFloor = new THREE.Mesh(
      new THREE.CircleGeometry(12, 64),
      new THREE.MeshBasicMaterial({
        color: 0x0b293d,
        transparent: true,
        opacity: 0.18,
        side: THREE.DoubleSide,
      }),
    );
    scanFloor.rotation.x = -Math.PI / 2;
    scanFloor.position.y = 0.008;
    scene.add(scanFloor);

    const maxVoxels = live ? 30000 : 2600;
    const voxelSize = 0.31;
    const voxelGeometry = new THREE.BoxGeometry(1, 1, 1);
    const voxelMaterial = new THREE.MeshStandardMaterial({
      color: 0x41bff3,
      roughness: 0.76,
      metalness: 0.06,
      transparent: true,
      opacity: 0.9,
    });
    const voxels = new THREE.InstancedMesh(
      voxelGeometry,
      voxelMaterial,
      maxVoxels,
    );
    voxels.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    voxels.count = 0;
    scene.add(voxels);

    const voxelKeys = new Set<string>();
    const matrix = new THREE.Matrix4();
    const color = new THREE.Color();
    let voxelCount = 0;

    const writeVoxel = (
      x: number,
      y: number,
      z: number,
      size: number,
      red: number,
      green: number,
      blue: number,
    ) => {
      if (voxelCount >= maxVoxels) return;
      matrix.makeScale(size * 0.92, size * 0.92, size * 0.92);
      matrix.setPosition(x, y, z);
      voxels.setMatrixAt(voxelCount, matrix);
      color.setRGB(red, green, blue);
      voxels.setColorAt(voxelCount, color);
      voxelCount += 1;
      voxels.count = voxelCount;
    };

    const addMockVoxel = (gx: number, gy: number, gz: number, tone = 0) => {
      if (voxelCount >= maxVoxels) return;
      const key = `${gx}:${gy}:${gz}`;
      if (voxelKeys.has(key)) return;
      voxelKeys.add(key);
      color.setHSL(0.53 + tone * 0.018, 0.72, 0.46 + gy * 0.018);
      writeVoxel(
        gx * voxelSize,
        gy * voxelSize + voxelSize / 2,
        gz * voxelSize,
        voxelSize,
        color.r,
        color.g,
        color.b,
      );
      voxels.instanceMatrix.needsUpdate = true;
      if (voxels.instanceColor) voxels.instanceColor.needsUpdate = true;
    };

    if (!live) {
      // The standalone design-preview page keeps its explicit mock scene.
      for (let index = -19; index <= 19; index += 1) {
        addMockVoxel(index, 0, -17, 0.1);
        if (index % 2 === 0) addMockVoxel(index, 1, -17, 0.12);
        if (index % 3 === 0) addMockVoxel(index, 2, -17, 0.15);
      }
      for (let index = -15; index <= 15; index += 1) {
        addMockVoxel(-21, 0, index, -0.05);
        if (index % 2 === 0) addMockVoxel(-21, 1, index, -0.02);
      }
      for (let column = 0; column < 5; column += 1) {
        for (let height = 0; height < 6; height += 1) {
          addMockVoxel(10 + column, height, 9, 0.18);
        }
      }
    }

    const robot = new THREE.Group();
    const robotBody = new THREE.Mesh(
      new THREE.BoxGeometry(0.88, 0.28, 0.62),
      new THREE.MeshStandardMaterial({
        color: 0x38a8ff,
        emissive: 0x0b3e67,
        emissiveIntensity: 0.45,
        roughness: 0.42,
      }),
    );
    robotBody.position.y = 0.34;
    robot.add(robotBody);
    const sensorTower = new THREE.Mesh(
      new THREE.CylinderGeometry(0.16, 0.18, 0.2, 20),
      new THREE.MeshStandardMaterial({ color: 0xe8f7ff, roughness: 0.3 }),
    );
    sensorTower.position.set(0, 0.62, 0.05);
    robot.add(sensorTower);
    const frontMark = new THREE.Mesh(
      new THREE.BoxGeometry(0.34, 0.08, 0.05),
      new THREE.MeshBasicMaterial({ color: 0x6ef4c5 }),
    );
    frontMark.position.set(0, 0.37, 0.335);
    robot.add(frontMark);
    for (const side of [-1, 1]) {
      for (const direction of [-1, 1]) {
        const wheel = new THREE.Mesh(
          new THREE.CylinderGeometry(0.14, 0.14, 0.1, 16),
          new THREE.MeshStandardMaterial({ color: 0x071019, roughness: 0.9 }),
        );
        wheel.rotation.z = Math.PI / 2;
        wheel.position.set(side * 0.48, 0.18, direction * 0.2);
        robot.add(wheel);
      }
    }
    scene.add(robot);

    const fovGeometry = new THREE.BufferGeometry();
    fovGeometry.setAttribute(
      "position",
      new THREE.Float32BufferAttribute(
        [0, 0.04, 0.3, -2.1, 0.04, 4.1, 2.1, 0.04, 4.1],
        3,
      ),
    );
    fovGeometry.setIndex([0, 1, 2]);
    fovGeometry.computeVertexNormals();
    const fov = new THREE.Mesh(
      fovGeometry,
      new THREE.MeshBasicMaterial({
        color: 0x35d9c2,
        transparent: true,
        opacity: 0.16,
        side: THREE.DoubleSide,
        depthWrite: false,
      }),
    );
    robot.add(fov);
    const fovOutline = new THREE.LineSegments(
      new THREE.EdgesGeometry(fovGeometry),
      new THREE.LineBasicMaterial({
        color: 0x61efd9,
        transparent: true,
        opacity: 0.68,
      }),
    );
    robot.add(fovOutline);

    const rayPositions: number[] = [];
    for (let index = 0; index < 34; index += 1) {
      const angle = (index / 34) * Math.PI * 2;
      const radius = 3.2 + (index % 5) * 0.23;
      rayPositions.push(
        0,
        0.54,
        0,
        Math.sin(angle) * radius,
        0.12,
        Math.cos(angle) * radius,
      );
    }
    const lidarGeometry = new THREE.BufferGeometry();
    lidarGeometry.setAttribute(
      "position",
      new THREE.Float32BufferAttribute(rayPositions, 3),
    );
    const lidarRays = new THREE.LineSegments(
      lidarGeometry,
      new THREE.LineBasicMaterial({
        color: 0xf3bd4f,
        transparent: true,
        opacity: 0.38,
      }),
    );
    robot.add(lidarRays);

    let rendererWidth = 0;
    let rendererHeight = 0;
    const resize = () => {
      const width = Math.max(1, Math.floor(host.clientWidth));
      const height = Math.max(1, Math.floor(host.clientHeight));
      if (width === rendererWidth && height === rendererHeight) return;
      rendererWidth = width;
      rendererHeight = height;
      renderer.setSize(width, height, false);
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
    };
    resize();

    let frame = 0;
    let previousVoxelTime = 0;
    let previousMetricTime = 0;
    let animationFrame = 0;
    const sessionStart = performance.now();
    let loadedRevision = -1;
    let framedLiveMap = false;

    const loadLiveSnapshot = (snapshot: LiveVoxelSnapshot) => {
      const revision = snapshot.revision ?? 0;
      if (revision === loadedRevision) return;
      loadedRevision = revision;
      voxelCount = 0;
      voxels.count = 0;
      if (
        !snapshot.ok
        || snapshot.encoding !== "base64-f32le-xyzsrgb"
        || !snapshot.data
      ) {
        voxels.instanceMatrix.needsUpdate = true;
        return;
      }

      const binary = window.atob(snapshot.data);
      const bytes = new Uint8Array(binary.length);
      for (let index = 0; index < binary.length; index += 1) {
        bytes[index] = binary.charCodeAt(index);
      }
      const view = new DataView(bytes.buffer);
      const stride = Math.max(7, snapshot.stride ?? 7);
      const floatCount = Math.floor(bytes.byteLength / 4);
      const pointCount = Math.min(
        maxVoxels,
        snapshot.voxel_count,
        Math.floor(floatCount / stride),
      );
      const bounds = new THREE.Box3();
      const point = new THREE.Vector3();
      for (let index = 0; index < pointCount; index += 1) {
        const offset = index * stride * 4;
        const rosX = view.getFloat32(offset, true);
        const rosY = view.getFloat32(offset + 4, true);
        const rosZ = view.getFloat32(offset + 8, true);
        const size = Math.max(0.01, view.getFloat32(offset + 12, true));
        const red = view.getFloat32(offset + 16, true);
        const green = view.getFloat32(offset + 20, true);
        const blue = view.getFloat32(offset + 24, true);
        // ROS: X/Y ground plane and Z up. Three.js: X/Z ground plane and Y up.
        const threeX = rosX;
        const threeY = rosZ;
        const threeZ = -rosY;
        writeVoxel(
          threeX,
          threeY,
          threeZ,
          size,
          Number.isFinite(red) ? red : 0.25,
          Number.isFinite(green) ? green : 0.72,
          Number.isFinite(blue) ? blue : 0.92,
        );
        point.set(threeX, threeY, threeZ);
        bounds.expandByPoint(point);
      }
      voxels.instanceMatrix.needsUpdate = true;
      if (voxels.instanceColor) voxels.instanceColor.needsUpdate = true;

      if (!framedLiveMap && !bounds.isEmpty()) {
        framedLiveMap = true;
        const center = bounds.getCenter(new THREE.Vector3());
        const size = bounds.getSize(new THREE.Vector3());
        const span = Math.max(size.x, size.z, 6);
        controls.target.copy(center);
        camera.position.set(
          center.x + span * 0.75,
          center.y + Math.max(6, span * 0.9),
          center.z + span * 0.9,
        );
        controls.update();
      }
    };

    const animate = (now: number) => {
      resize();
      let robotX = 0;
      let robotZ = 0;
      let heading = 0;
      if (live) {
        const currentSnapshot = voxelSnapshotRef.current;
        if (currentSnapshot) {
          loadLiveSnapshot(currentSnapshot);
          robotX = currentSnapshot.robot.x;
          robotZ = -currentSnapshot.robot.y;
          heading = Math.PI / 2 + currentSnapshot.robot.yaw;
        }
      } else {
        const elapsed = (now - sessionStart) / 1000;
        const pathTime = activeRef.current ? elapsed : 0;
        robotX = Math.sin(pathTime * 0.34) * 4.7 + Math.sin(pathTime * 0.11);
        robotZ = Math.cos(pathTime * 0.27) * 3.8 + Math.sin(pathTime * 0.18) * 1.4;
        const nextX = Math.sin((pathTime + 0.05) * 0.34) * 4.7 + Math.sin((pathTime + 0.05) * 0.11);
        const nextZ = Math.cos((pathTime + 0.05) * 0.27) * 3.8 + Math.sin((pathTime + 0.05) * 0.18) * 1.4;
        heading = Math.atan2(nextX - robotX, nextZ - robotZ);
      }
      robot.position.set(robotX, 0.02, robotZ);
      robot.rotation.y = heading;
      fov.visible = showFovRef.current;
      fovOutline.visible = showFovRef.current;
      // The live endpoint currently carries occupied OctoMap cells and the
      // robot pose, not raw LaserScan ranges. Never present decorative rays
      // as live sensor data.
      lidarRays.visible = !live && showLidarRef.current;

      if (!live && activeRef.current && now - previousVoxelTime > 115) {
        previousVoxelTime = now;
        const baseX = Math.round(robotX / voxelSize);
        const baseZ = Math.round(robotZ / voxelSize);
        for (let count = 0; count < 7; count += 1) {
          const side = count % 2 === 0 ? -1 : 1;
          const radius = 6 + Math.floor(Math.random() * 9);
          const localAngle = heading + side * (0.62 + Math.random() * 0.95);
          const gx = baseX + Math.round(Math.sin(localAngle) * radius);
          const gz = baseZ + Math.round(Math.cos(localAngle) * radius);
          const wallHeight = Math.random() > 0.72 ? 5 : 2 + Math.floor(Math.random() * 3);
          for (let height = 0; height < wallHeight; height += 1) {
            if (Math.random() > 0.18) addMockVoxel(gx, height, gz, side * 0.08);
          }
        }
      }

      if (now - previousMetricTime > 420) {
        previousMetricTime = now;
        onMetricsRef.current({
          voxelCount,
          coverage: Math.min(99, Math.round((voxelCount / maxVoxels) * 100)),
          robotX,
          robotZ,
        });
      }

      frame += 1;
      if (!live) lidarRays.rotation.y = frame * 0.014;
      controls.update();
      renderer.render(scene, camera);
      animationFrame = window.requestAnimationFrame(animate);
    };
    animationFrame = window.requestAnimationFrame(animate);

    return () => {
      window.cancelAnimationFrame(animationFrame);
      controls.dispose();
      scene.traverse((object) => {
        if (!(object instanceof THREE.Mesh || object instanceof THREE.LineSegments)) return;
        object.geometry.dispose();
        const materials = Array.isArray(object.material)
          ? object.material
          : [object.material];
        materials.forEach((material) => material.dispose());
      });
      renderer.dispose();
    };
  }, [canvasRef, live, resetSignal]);

  return (
    <div ref={hostRef} className="slam-viewer-canvas-wrap">
      <canvas ref={canvasRef} aria-label="实时三维体素地图" />
    </div>
  );
}

function MapThumbnail({ map }: { map: MapRecord }) {
  if (map.preview) {
    return <img src={map.preview} alt={`${map.name} 地图预览`} />;
  }
  return (
    <div className="slam-generated-thumb" aria-label={`${map.name} 模拟体素缩略图`}>
      {Array.from({ length: 28 }, (_, index) => (
        <i
          key={index}
          style={{
            left: `${8 + ((index * 23) % 78)}%`,
            top: `${11 + ((index * 31) % 69)}%`,
            opacity: 0.42 + (index % 5) * 0.1,
          }}
        />
      ))}
      <b />
    </div>
  );
}

export function AutonomousMappingWorkspace({ onReturnToNavigation }: Props) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const uploadRef = useRef<HTMLInputElement>(null);
  const [status, setStatus] = useState<MappingStatus>("idle");
  const [duration, setDuration] = useState(0);
  const [startedAt, setStartedAt] = useState<number | null>(null);
  const [showFov, setShowFov] = useState(true);
  const [showLidar, setShowLidar] = useState(false);
  const [resetSignal, setResetSignal] = useState(0);
  const [metrics, setMetrics] = useState<ViewerMetrics>({
    voxelCount: 0,
    coverage: 0,
    robotX: 0,
    robotZ: 0,
  });
  const [maps, setMaps] = useState<MapRecord[]>(MOCK_MAPS);
  const [modalOpen, setModalOpen] = useState(false);
  const [mapName, setMapName] = useState("");
  const [preview, setPreview] = useState<string | undefined>();
  const [busyMapId, setBusyMapId] = useState<string | null>(null);
  const [notice, setNotice] = useState("Mock 数据已连接，可直接体验完整建图流程");

  useEffect(() => {
    if (status !== "mapping" || startedAt === null) return;
    const updateDuration = () => {
      setDuration(Math.floor((Date.now() - startedAt) / 1000));
    };
    updateDuration();
    const timer = window.setInterval(updateDuration, 1000);
    return () => window.clearInterval(timer);
  }, [startedAt, status]);

  const handleMetrics = useCallback((nextMetrics: ViewerMetrics) => {
    setMetrics(nextMetrics);
  }, []);

  const storageTotal = useMemo(
    () => maps.reduce((total, map) => total + map.sizeMb, 0),
    [maps],
  );

  const startMapping = async () => {
    setNotice("正在创建 SLAM 会话并检查传感器…");
    setResetSignal((value) => value + 1);
    setDuration(0);
    setStartedAt(Date.now());
    setStatus("mapping");
    await sendMockMappingRequest("/api/mapping/start", {
      mode: "frontier_exploration",
      voxelResolution: 0.08,
      sources: ["lidar", "rgbd", "wheel_odom", "imu"],
    });
    setNotice("自主探索已启动 · 正在寻找下一个未知区域");
  };

  const openSaveDialog = () => {
    setStatus("saving");
    setMapName(`自主建图 ${new Intl.DateTimeFormat("zh-CN", {
      month: "2-digit",
      day: "2-digit",
      hour: "2-digit",
      minute: "2-digit",
      hour12: false,
    }).format(new Date())}`);
    setPreview(undefined);
    setModalOpen(true);
    setNotice("探索已暂停，请确认地图名称与预览");
  };

  const closeSaveDialog = () => {
    setModalOpen(false);
    setStatus("mapping");
    setNotice("已返回探索建图，体素融合继续运行");
  };

  const generatePreview = () => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    setPreview(canvas.toDataURL("image/jpeg", 0.78));
    setNotice("已从当前 3D 视角生成地图预览");
  };

  const importPreview = (file?: File) => {
    if (!file) return;
    if (!file.type.startsWith("image/")) {
      setNotice("请选择 PNG、JPEG 或 WebP 图片");
      return;
    }
    const reader = new FileReader();
    reader.onload = () => {
      if (typeof reader.result === "string") {
        setPreview(reader.result);
        setNotice("自定义地图预览已加载");
      }
    };
    reader.readAsDataURL(file);
  };

  const saveMap = async () => {
    const normalizedName = mapName.trim();
    if (!normalizedName) {
      setNotice("请输入地图名称后再保存");
      return;
    }
    setStatus("saving");
    setNotice("正在保存 OctoMap、二维栅格图与预览…");
    const generatedPreview =
      preview ?? canvasRef.current?.toDataURL("image/jpeg", 0.78);
    await sendMockMappingRequest("/api/mapping/finish", {
      name: normalizedName,
      formats: ["ot", "yaml", "pgm"],
      generateThumbnail: !preview,
    });
    const newMap: MapRecord = {
      id: `octomap-${Date.now()}`,
      name: normalizedName,
      createdAt: new Date().toISOString(),
      sizeMb: Number((6.8 + metrics.voxelCount / 22000).toFixed(1)),
      voxelCount: Math.max(metrics.voxelCount * 94, 24360),
      resolution: 0.08,
      deployed: false,
      preview: generatedPreview,
    };
    setMaps((current) => [newMap, ...current]);
    setStatus("saved");
    setModalOpen(false);
    setStartedAt(null);
    setNotice(`“${normalizedName}”已保存到地图仓库`);
  };

  const discardMapping = async () => {
    setNotice("正在清空本次建图缓存…");
    await sendMockMappingRequest("/api/mapping/discard", {
      clearOctomap: true,
      clearSlamSession: true,
    });
    setStatus("idle");
    setDuration(0);
    setStartedAt(null);
    setResetSignal((value) => value + 1);
    setNotice("当前建图缓存已清空，历史地图不受影响");
  };

  const deployMap = async (map: MapRecord) => {
    setBusyMapId(map.id);
    setNotice(`正在把“${map.name}”加载到机器人内存…`);
    await sendMockMappingRequest(`/api/maps/3d/${map.id}/activate`, {
      loadOctomap: true,
      loadGridMap: true,
      resetLocalization: true,
    });
    setMaps((current) =>
      current.map((item) => ({ ...item, deployed: item.id === map.id })),
    );
    setBusyMapId(null);
    setNotice(`“${map.name}”部署完成，机器人正在重新定位`);
  };

  const deleteMap = async (map: MapRecord) => {
    if (map.deployed) return;
    setBusyMapId(map.id);
    setNotice(`正在删除“${map.name}”…`);
    await sendMockMappingRequest(`/api/maps/3d/${map.id}/delete`);
    setMaps((current) => current.filter((item) => item.id !== map.id));
    setBusyMapId(null);
    setNotice(`“${map.name}”已从地图仓库删除`);
  };

  const isMapping = status === "mapping";
  const statusCopy = STATUS_COPY[status];

  return (
    <section className="slam-workspace">
      <style>{`
        .slam-workspace{--slam-blue:#1686f7;--slam-cyan:#39c9f1;--slam-green:#2ac99b;--slam-red:#e45865;--slam-ink:#14273a;--slam-muted:#77889a;min-height:calc(100vh - 74px);padding:20px 22px 28px;background:#f3f6f9;color:var(--slam-ink)}
        .slam-workspace *{box-sizing:border-box}.slam-workspace button,.slam-workspace input{font:inherit}.slam-head{display:flex;align-items:flex-end;justify-content:space-between;gap:24px;margin-bottom:16px}.slam-head-copy span{font-size:9px;font-weight:850;letter-spacing:1.8px;color:#2174d3}.slam-head-copy h1{margin:5px 0 5px;font-size:23px;letter-spacing:-.6px}.slam-head-copy p{margin:0;color:var(--slam-muted);font-size:11px}.slam-mode-tabs{display:flex;padding:4px;border:1px solid #dce4ec;border-radius:10px;background:#fff;box-shadow:0 4px 14px #24445f0b}.slam-mode-tabs button{height:34px;padding:0 15px;border:0;border-radius:7px;background:transparent;color:#738396;font-size:10px;font-weight:750;cursor:pointer}.slam-mode-tabs button.active{background:#eaf4ff;color:#156fcf;box-shadow:inset 0 0 0 1px #afd1f5}.slam-layout{display:grid;grid-template-columns:250px minmax(500px,1fr) 300px;gap:14px;min-height:650px}.slam-panel{border:1px solid #dde5ec;border-radius:13px;background:#fff;box-shadow:0 12px 34px #213e5710;overflow:hidden}.slam-panel-head{display:flex;align-items:center;justify-content:space-between;gap:10px;padding:15px 16px;border-bottom:1px solid #e8edf2}.slam-panel-head>div{display:flex;align-items:center;gap:10px}.slam-panel-icon{width:32px;height:32px;border-radius:9px;display:grid;place-items:center;background:#eaf5ff;color:#1376da;font-size:16px}.slam-panel-head h2,.slam-panel-head p{margin:0}.slam-panel-head h2{font-size:12px}.slam-panel-head p{margin-top:3px;color:#8a98a7;font-size:8px}.slam-mock-badge{padding:4px 7px;border-radius:12px;background:#edf1f5;color:#7d8997;font-size:7px;font-weight:850;letter-spacing:.7px}.mapping-controller{display:flex;flex-direction:column}.mapping-state-card{margin:15px;padding:14px;border:1px solid #dce7ef;border-radius:11px;background:linear-gradient(145deg,#f8fbfd,#eef5fa)}.mapping-state-card.mapping{border-color:#8fd7cb;background:linear-gradient(145deg,#f0fbf8,#e4f7f2)}.mapping-state-card.saving{border-color:#f0cc8e;background:linear-gradient(145deg,#fffaf0,#fff3d9)}.mapping-state-card.saved{border-color:#9cdcc8;background:#ecfaf5}.mapping-state-top{display:flex;align-items:center;justify-content:space-between}.mapping-state-label{display:flex;align-items:center;gap:7px;font-size:11px;font-weight:850}.mapping-state-label i{width:8px;height:8px;border-radius:50%;background:#93a3b2}.mapping .mapping-state-label i{background:#21c997;box-shadow:0 0 0 5px #21c99718;animation:slam-pulse 1.7s infinite}.saving .mapping-state-label i{background:#e7aa3f;animation:slam-pulse 1.1s infinite}.saved .mapping-state-label i{background:#21b986}.mapping-clock{font-variant-numeric:tabular-nums;font-size:14px;font-weight:800;color:#1c547e}.mapping-state-card p{margin:10px 0 0;color:#708496;font-size:8px;line-height:1.55}.mapping-progress{height:5px;margin-top:13px;border-radius:4px;background:#dfe8ee;overflow:hidden}.mapping-progress i{display:block;height:100%;border-radius:inherit;background:linear-gradient(90deg,#1788f7,#36d3bd);transition:width .35s}.mapping-stat-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin:0 15px 15px}.mapping-stat-grid>div{padding:10px;border:1px solid #e4eaf0;border-radius:9px;background:#fafcfd}.mapping-stat-grid small,.mapping-stat-grid strong{display:block}.mapping-stat-grid small{font-size:7px;color:#8a98a7}.mapping-stat-grid strong{margin-top:5px;font-size:12px}.mapping-stat-grid strong span{font-size:7px;color:#8a98a7;margin-left:3px}.mapping-actions{display:grid;gap:8px;padding:0 15px}.mapping-actions button{min-height:39px;border-radius:8px;font-size:9px;font-weight:800;cursor:pointer;transition:.18s}.mapping-actions button:disabled{opacity:.42;cursor:not-allowed;transform:none}.mapping-start{border:0;background:linear-gradient(90deg,#147be8,#1aa7ee);color:#fff;box-shadow:0 7px 18px #1788e52e}.mapping-save{border:1px solid #9dddc9;background:#edfaf6;color:#138364}.mapping-discard{border:1px solid #efc4c7;background:#fff8f8;color:#c84e59}.mapping-actions button:hover:not(:disabled){transform:translateY(-1px)}.mapping-checklist{margin:auto 15px 15px;padding:13px;border-radius:10px;background:#f4f7fa}.mapping-checklist strong{display:block;font-size:8px}.mapping-checklist ul{list-style:none;margin:9px 0 0;padding:0}.mapping-checklist li{display:flex;align-items:center;justify-content:space-between;margin-top:7px;color:#6e8091;font-size:8px}.mapping-checklist li:before{content:"✓";width:16px;height:16px;margin-right:7px;border-radius:50%;display:grid;place-items:center;background:#def5ec;color:#168961;font-size:7px;font-weight:900}.mapping-checklist li span{margin-left:auto;color:#269c79;font-size:7px;font-weight:800}.slam-viewer{position:relative;min-height:650px;background:#07111e}.slam-viewer .slam-panel-head{position:absolute;z-index:5;left:0;right:0;top:0;border-color:#ffffff18;background:linear-gradient(180deg,#07111ef2,#07111e99);color:#eef8ff}.slam-viewer .slam-panel-head p{color:#83a1b6}.slam-viewer .slam-panel-icon{background:#1a405b;color:#56d7ff}.slam-viewer-canvas-wrap{position:absolute;inset:0}.slam-viewer-canvas-wrap canvas{display:block;width:100%;height:100%;touch-action:none}.slam-viewer-hud{position:absolute;z-index:5;left:15px;right:15px;bottom:15px;display:flex;align-items:flex-end;justify-content:space-between;gap:14px;pointer-events:none}.viewer-metrics{display:flex;gap:6px}.viewer-metrics span{min-width:78px;padding:8px 10px;border:1px solid #ffffff1c;border-radius:8px;background:#071522cf;color:#8ea7b9;backdrop-filter:blur(10px);font-size:7px}.viewer-metrics strong{display:block;margin-top:3px;color:#ecf8ff;font-size:10px;font-variant-numeric:tabular-nums}.viewer-help{padding:8px 10px;border:1px solid #ffffff1a;border-radius:8px;background:#071522c9;color:#8ea7b9;font-size:7px;backdrop-filter:blur(10px)}.viewer-controls{position:absolute;z-index:6;right:14px;top:70px;display:grid;gap:7px}.viewer-switch{display:flex;align-items:center;gap:8px;padding:7px 9px;border:1px solid #ffffff1a;border-radius:8px;background:#071522d8;color:#bad0de;font-size:8px;backdrop-filter:blur(9px);cursor:pointer}.viewer-switch input{position:absolute;opacity:0;pointer-events:none}.viewer-switch i{position:relative;width:27px;height:15px;border-radius:9px;background:#304755;transition:.2s}.viewer-switch i:after{content:"";position:absolute;left:2px;top:2px;width:11px;height:11px;border-radius:50%;background:#8ca2b1;transition:.2s}.viewer-switch input:checked+i{background:#167fd3}.viewer-switch input:checked+i:after{left:14px;background:#e8f8ff}.mapping-library{display:flex;min-width:0;flex-direction:column}.map-library-meta{padding:12px 15px;border-bottom:1px solid #e9edf1;background:#fafcfd;color:#7f8d9c;font-size:8px}.map-library-meta strong{color:#31506b}.stored-map-list{flex:1;min-height:0;overflow:auto;padding:10px}.stored-map-card{padding:9px;margin-bottom:8px;border:1px solid #e1e8ee;border-radius:10px;background:#fff;transition:.18s}.stored-map-card:hover{border-color:#b6d3ee;box-shadow:0 7px 18px #17446b10}.stored-map-main{display:grid;grid-template-columns:72px minmax(0,1fr);gap:10px}.stored-map-thumb{height:58px;border-radius:7px;overflow:hidden;background:#092033}.stored-map-thumb img,.slam-generated-thumb{display:block;width:100%;height:100%;object-fit:cover}.slam-generated-thumb{position:relative;background:linear-gradient(145deg,#071a2a,#0d3850)}.slam-generated-thumb:before{content:"";position:absolute;inset:0;background-image:linear-gradient(#59c7ef14 1px,transparent 1px),linear-gradient(90deg,#59c7ef14 1px,transparent 1px);background-size:10px 10px}.slam-generated-thumb i{position:absolute;width:5px;height:5px;border-radius:1px;background:#4fd1f5;box-shadow:3px -2px 0 #257fa4}.slam-generated-thumb b{position:absolute;left:48%;top:48%;width:7px;height:10px;border-radius:2px;background:#58e5c0;box-shadow:0 0 8px #58e5c0}.stored-map-copy{min-width:0}.stored-map-title{display:flex;align-items:center;justify-content:space-between;gap:5px}.stored-map-title strong{overflow:hidden;text-overflow:ellipsis;white-space:nowrap;font-size:9px}.stored-map-title em{flex:0 0 auto;padding:3px 5px;border-radius:8px;background:#dff6ed;color:#12805d;font-style:normal;font-size:6px;font-weight:850}.stored-map-copy>span{display:block;margin-top:5px;color:#8291a0;font-size:7px}.stored-map-stats{display:flex;gap:8px;margin-top:7px}.stored-map-stats span{color:#62788c;font-size:7px}.stored-map-stats b{color:#274967}.stored-map-actions{display:grid;grid-template-columns:1fr 62px;gap:6px;margin-top:9px}.stored-map-actions button{height:29px;border-radius:6px;font-size:7px;font-weight:800;cursor:pointer}.deploy-map{border:0;background:#eaf4ff;color:#176fcf}.deploy-map.active{background:#e4f7f0;color:#14815f}.delete-map{border:1px solid #efced0;background:#fff;color:#cb5861}.stored-map-actions button:disabled{opacity:.45;cursor:not-allowed}.slam-notice{margin:0 10px 10px;padding:9px 10px;border-radius:8px;background:#eef5fb;color:#56718a;font-size:8px;line-height:1.5}.slam-save-backdrop{position:fixed;z-index:1000;inset:0;display:grid;place-items:center;padding:20px;background:#06111caa;backdrop-filter:blur(5px)}.slam-save-modal{width:min(560px,100%);border:1px solid #d9e4ec;border-radius:15px;background:#fff;box-shadow:0 28px 80px #06111c77;overflow:hidden}.slam-modal-head{display:flex;align-items:center;justify-content:space-between;padding:17px 19px;border-bottom:1px solid #e8edf1}.slam-modal-head h2,.slam-modal-head p{margin:0}.slam-modal-head h2{font-size:14px}.slam-modal-head p{margin-top:4px;color:#8493a2;font-size:8px}.slam-modal-head button{width:29px;height:29px;border:0;border-radius:7px;background:#f0f3f6;color:#718191;cursor:pointer}.slam-modal-body{display:grid;grid-template-columns:1fr 190px;gap:16px;padding:18px 19px}.slam-modal-form label{display:block;color:#647789;font-size:8px;font-weight:750}.slam-modal-form input[type=text]{display:block;width:100%;height:38px;margin-top:7px;padding:0 11px;border:1px solid #dce4eb;border-radius:8px;color:#213a50;font-size:10px;outline:0}.slam-modal-form input[type=text]:focus{border-color:#68aae8;box-shadow:0 0 0 3px #2786df16}.slam-save-summary{display:grid;grid-template-columns:1fr 1fr;gap:7px;margin-top:14px}.slam-save-summary div{padding:10px;border-radius:8px;background:#f4f7f9}.slam-save-summary small,.slam-save-summary strong{display:block}.slam-save-summary small{font-size:7px;color:#8a98a6}.slam-save-summary strong{margin-top:4px;font-size:9px}.slam-preview-editor{min-width:0}.slam-preview-frame{height:118px;border:1px dashed #b9c8d5;border-radius:9px;overflow:hidden;display:grid;place-items:center;background:#f2f6f8;color:#7c8e9e;text-align:center}.slam-preview-frame img{width:100%;height:100%;object-fit:cover}.slam-preview-frame span{font-size:8px;line-height:1.5}.slam-preview-actions{display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-top:7px}.slam-preview-actions button{height:30px;border:1px solid #dce5ec;border-radius:6px;background:#fff;color:#597084;font-size:7px;cursor:pointer}.slam-modal-foot{display:flex;align-items:center;justify-content:flex-end;gap:8px;padding:13px 19px;border-top:1px solid #e8edf1;background:#fafcfd}.slam-modal-foot button{height:35px;padding:0 15px;border-radius:7px;font-size:8px;font-weight:800;cursor:pointer}.slam-modal-cancel{border:1px solid #dce4ea;background:#fff;color:#6f7f8e}.slam-modal-save{border:0;background:#177ee8;color:#fff;box-shadow:0 6px 14px #177ee82c}.slam-modal-save:disabled{opacity:.4;cursor:not-allowed}@keyframes slam-pulse{50%{transform:scale(1.18);box-shadow:0 0 0 7px #21c9970a}}
        @media(max-width:1280px){.slam-layout{grid-template-columns:230px minmax(460px,1fr)}.mapping-library{grid-column:1/-1}.stored-map-list{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}.stored-map-card{margin:0}.slam-notice{margin-top:10px}}
        @media(max-width:850px){.slam-workspace{padding:14px}.slam-head{display:block}.slam-mode-tabs{margin-top:13px;width:max-content}.slam-layout{display:block}.slam-panel{margin-bottom:12px}.slam-viewer{min-height:520px}.mapping-controller{min-height:520px}.stored-map-list{grid-template-columns:1fr 1fr}.slam-modal-body{grid-template-columns:1fr}}
        @media(max-width:560px){.slam-workspace{padding:10px}.slam-mode-tabs{width:100%}.slam-mode-tabs button{flex:1;padding:0 8px}.slam-viewer{min-height:470px}.viewer-metrics span:nth-child(3){display:none}.viewer-help{display:none}.stored-map-list{grid-template-columns:1fr}.slam-modal-body{display:block}.slam-preview-editor{margin-top:15px}}
      `}</style>

      <header className="slam-head">
        <div className="slam-head-copy">
          <span>AUTONOMOUS MAPPING</span>
          <h1>自主 SLAM 3D 建图与存储管理</h1>
          <p>前沿探索、激光定位、RGB-D 体素融合与地图版本管理</p>
        </div>
        <div className="slam-mode-tabs" role="tablist" aria-label="机器人工作模式">
          <button type="button" role="tab" aria-selected="false" onClick={onReturnToNavigation}>日常巡检导航</button>
          <button type="button" role="tab" aria-selected="true" className="active">自主 3D 建图</button>
        </div>
      </header>

      <div className="slam-layout">
        <aside className="slam-panel mapping-controller">
          <div className="slam-panel-head">
            <div><span className="slam-panel-icon">◎</span><div><h2>建图控制</h2><p>Mapping Controller</p></div></div>
            <span className="slam-mock-badge">MOCK</span>
          </div>
          <div className={`mapping-state-card ${status}`}>
            <div className="mapping-state-top">
              <span className="mapping-state-label"><i />{statusCopy.label}</span>
              <strong className="mapping-clock">{formatDuration(duration)}</strong>
            </div>
            <p>{statusCopy.detail}</p>
            <div className="mapping-progress"><i style={{ width: `${metrics.coverage}%` }} /></div>
          </div>
          <div className="mapping-stat-grid">
            <div><small>当前体素</small><strong>{metrics.voxelCount.toLocaleString()}<span>cells</span></strong></div>
            <div><small>探索覆盖</small><strong>{metrics.coverage}<span>%</span></strong></div>
            <div><small>体素分辨率</small><strong>8<span>cm</span></strong></div>
            <div><small>机器人坐标</small><strong>{metrics.robotX.toFixed(1)}<span>, {metrics.robotZ.toFixed(1)} m</span></strong></div>
          </div>
          <div className="mapping-actions">
            <button type="button" className="mapping-start" onClick={startMapping} disabled={status === "mapping" || status === "saving"}>{status === "mapping" ? "●  探索任务运行中" : "▶  开始自主探索建图"}</button>
            <button type="button" className="mapping-save" onClick={openSaveDialog} disabled={!isMapping}>■  结束并保存地图</button>
            <button type="button" className="mapping-discard" onClick={discardMapping} disabled={status !== "mapping" && status !== "saving"}>×  放弃当前建图</button>
          </div>
          <div className="mapping-checklist">
            <strong>建图链路健康状态</strong>
            <ul>
              <li>激光 SLAM <span>10 Hz</span></li>
              <li>RGB-D 建图点云 <span>3 Hz</span></li>
              <li>轮速 + IMU EKF <span>30 Hz</span></li>
              <li>OctoMap 融合 <span>正常</span></li>
            </ul>
          </div>
        </aside>

        <main className="slam-panel slam-viewer">
          <div className="slam-panel-head">
            <div><span className="slam-panel-icon">⬡</span><div><h2>3D 体素地图实时预览</h2><p>拖动旋转 · 滚轮缩放 · 右键平移</p></div></div>
            <span className="slam-mock-badge">LIVE SCENE</span>
          </div>
          <VoxelMapScene
            active={isMapping}
            resetSignal={resetSignal}
            showFov={showFov}
            showLidar={showLidar}
            canvasRef={canvasRef}
            onMetrics={handleMetrics}
          />
          <div className="viewer-controls">
            <label className="viewer-switch"><input type="checkbox" checked={showFov} onChange={(event) => setShowFov(event.target.checked)} /><i />显示机器人视场角 FOV</label>
            <label className="viewer-switch"><input type="checkbox" checked={showLidar} onChange={(event) => setShowLidar(event.target.checked)} /><i />显示激光雷达射线</label>
          </div>
          <div className="slam-viewer-hud">
            <div className="viewer-metrics">
              <span>MAP FRAME<strong>map</strong></span>
              <span>VOXEL SIZE<strong>0.08 m</strong></span>
              <span>SENSOR RANGE<strong>6.0 m</strong></span>
            </div>
            <span className="viewer-help">左键旋转　滚轮缩放　右键平移</span>
          </div>
        </main>

        <aside className="slam-panel mapping-library">
          <div className="slam-panel-head">
            <div><span className="slam-panel-icon">▤</span><div><h2>地图存储仓库</h2><p>Map Library</p></div></div>
            <span className="slam-mock-badge">{maps.length} MAPS</span>
          </div>
          <div className="map-library-meta">本地地图占用 <strong>{storageTotal.toFixed(1)} MB</strong> · `.ot + .yaml + .pgm`</div>
          <div className="stored-map-list">
            {maps.map((map) => (
              <article className="stored-map-card" key={map.id}>
                <div className="stored-map-main">
                  <div className="stored-map-thumb"><MapThumbnail map={map} /></div>
                  <div className="stored-map-copy">
                    <div className="stored-map-title"><strong title={map.name}>{map.name}</strong>{map.deployed && <em>车辆使用中</em>}</div>
                    <span>{formatMapDate(map.createdAt)} 建图</span>
                    <div className="stored-map-stats"><span><b>{map.sizeMb.toFixed(1)}</b> MB</span><span><b>{(map.voxelCount / 1000).toFixed(0)}k</b> 体素</span><span><b>{Math.round(map.resolution * 100)}</b> cm</span></div>
                  </div>
                </div>
                <div className="stored-map-actions">
                  <button type="button" className={`deploy-map ${map.deployed ? "active" : ""}`} onClick={() => deployMap(map)} disabled={busyMapId !== null || map.deployed}>{busyMapId === map.id ? "正在加载…" : map.deployed ? "✓ 已部署" : "应用此地图 / 部署"}</button>
                  <button type="button" className="delete-map" onClick={() => deleteMap(map)} disabled={busyMapId !== null || map.deployed}>删除</button>
                </div>
              </article>
            ))}
          </div>
          <p className="slam-notice" role="status">{notice}</p>
        </aside>
      </div>

      {modalOpen && (
        <div className="slam-save-backdrop" role="presentation" onMouseDown={(event) => { if (event.target === event.currentTarget) closeSaveDialog(); }}>
          <div className="slam-save-modal" role="dialog" aria-modal="true" aria-labelledby="slam-save-title">
            <div className="slam-modal-head">
              <div><h2 id="slam-save-title">结束并保存地图</h2><p>保存彩色 OctoMap、二维导航栅格和本次建图预览</p></div>
              <button type="button" onClick={closeSaveDialog} aria-label="关闭保存地图窗口">×</button>
            </div>
            <div className="slam-modal-body">
              <div className="slam-modal-form">
                <label>地图名称<input type="text" autoFocus value={mapName} maxLength={48} placeholder="例如：一号管廊 2026-07" onChange={(event) => setMapName(event.target.value)} /></label>
                <div className="slam-save-summary">
                  <div><small>建图持续时间</small><strong>{formatDuration(duration)}</strong></div>
                  <div><small>当前体素</small><strong>{metrics.voxelCount.toLocaleString()} cells</strong></div>
                  <div><small>地图坐标系</small><strong>map</strong></div>
                  <div><small>保存格式</small><strong>.ot + 2D map</strong></div>
                </div>
              </div>
              <div className="slam-preview-editor">
                <div className="slam-preview-frame">{preview ? <img src={preview} alt="即将保存的地图预览" /> : <span>尚未生成预览<br />可使用当前 3D 视角或上传图片</span>}</div>
                <input ref={uploadRef} type="file" accept="image/png,image/jpeg,image/webp" hidden onChange={(event) => importPreview(event.target.files?.[0])} />
                <div className="slam-preview-actions"><button type="button" onClick={generatePreview}>生成预览</button><button type="button" onClick={() => uploadRef.current?.click()}>上传图片</button></div>
              </div>
            </div>
            <div className="slam-modal-foot">
              <button type="button" className="slam-modal-cancel" onClick={closeSaveDialog}>返回继续建图</button>
              <button type="button" className="slam-modal-save" onClick={saveMap} disabled={!mapName.trim()}>保存到地图仓库</button>
            </div>
          </div>
        </div>
      )}
    </section>
  );
}

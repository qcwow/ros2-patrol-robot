export type MapSource = "preset" | "generated" | "imported";
export type SceneObjectType = "obstacle" | "device";

export type Waypoint = {
  id: number;
  name: string;
  x: number;
  y: number;
  dwell: number;
};

export type SceneObject = {
  id: string;
  type: SceneObjectType;
  name: string;
  x: number;
  y: number;
  width: number;
  depth: number;
  height: number;
};

export type OccupancyLayer = {
  width: number;
  height: number;
  resolution: number;
  originX: number;
  originY: number;
  /** One occupied/free bit per PGM pixel, packed row-major and base64 encoded. */
  data: string;
};

export type PatrolMap = {
  id: string;
  name: string;
  description: string;
  source: MapSource;
  seed?: string;
  bounds: { minX: number; minY: number; width: number; height: number };
  resolution: number;
  objects: SceneObject[];
  waypoints: Waypoint[];
  occupancy?: OccupancyLayer;
  createdAt: string;
  updatedAt: string;
};

export type WaypointSafetyIssue = {
  waypoint: Waypoint;
  reason: string;
};

// The simulated footprint is 0.58 m x 0.46 m. Its circumscribed radius is
// about 0.37 m; the extra margin keeps goals out of Nav2's inflated cells.
export const WAYPOINT_SAFETY_RADIUS = 0.45;

const now = "2026-07-15T00:00:00.000Z";

export const DEFAULT_MAPS: PatrolMap[] = [
  {
    id: "pipeline-demo",
    name: "管廊综合测试区",
    description: "标准管廊、控制柜与维护间场景",
    source: "preset",
    bounds: { minX: -8, minY: -6, width: 16, height: 12 },
    resolution: 0.5,
    objects: [
      { id: "pipe-a", type: "device", name: "A区管道架", x: -2.75, y: 1.5, width: 1, depth: 4, height: 1.2 },
      { id: "pipe-b", type: "device", name: "B区管道架", x: 2.25, y: -2, width: 1, depth: 4, height: 1.2 },
      { id: "cabinet", type: "device", name: "控制柜", x: 0.25, y: 1, width: 1.5, depth: 1, height: 1.4 },
      { id: "north-equipment", type: "device", name: "北侧设备区", x: 4.7, y: 3.7, width: 2.3, depth: 1, height: 0.8 },
      { id: "maintenance-room", type: "device", name: "维护间", x: -5.4, y: 0.2, width: 1.7, depth: 1.3, height: 0.9 },
    ],
    waypoints: [
      { id: 1, name: "起点东侧", x: -4.8, y: -3.8, dwell: 2 },
      { id: 2, name: "A区管道北侧", x: -4.6, y: 3.8, dwell: 3 },
      { id: 3, name: "控制柜检查点", x: 0.2, y: 3.2, dwell: 3 },
      { id: 4, name: "B区管道东侧", x: 4.6, y: -1, dwell: 3 },
      { id: 5, name: "返回区", x: -5.8, y: -4, dwell: 2 },
    ],
    createdAt: now,
    updatedAt: now,
  },
  {
    id: "narrow-corridor",
    name: "狭窄通道测试",
    description: "连续窄通道与急转弯组合",
    source: "preset",
    bounds: { minX: -8, minY: -6, width: 16, height: 12 },
    resolution: 0.25,
    objects: [
      { id: "wall-n1", type: "obstacle", name: "北侧隔离墙", x: -1.7, y: 2.2, width: 8.2, depth: 0.55, height: 1.4 },
      { id: "wall-s1", type: "obstacle", name: "南侧隔离墙", x: 1.4, y: -1.1, width: 8.5, depth: 0.55, height: 1.4 },
      { id: "wall-e1", type: "obstacle", name: "东侧折返墙", x: 5.35, y: 2.75, width: 0.55, depth: 3.8, height: 1.4 },
      { id: "pump-n", type: "device", name: "北侧泵组", x: -5.7, y: 4.2, width: 1.5, depth: 1.1, height: 1.1 },
      { id: "valve-s", type: "device", name: "南侧阀组", x: 5.4, y: -4.1, width: 1.3, depth: 1, height: 1.0 },
    ],
    waypoints: [
      { id: 1, name: "通道入口", x: -6, y: -4, dwell: 2 },
      { id: 2, name: "第一折点", x: -4.8, y: 0.3, dwell: 2 },
      { id: 3, name: "中段检查点", x: 2.8, y: 0.4, dwell: 3 },
      { id: 4, name: "东侧折返", x: 4.2, y: 3.8, dwell: 2 },
    ],
    createdAt: now,
    updatedAt: now,
  },
  {
    id: "dense-equipment",
    name: "高密度设备区",
    description: "设备密集、局部遮挡与多巡检目标",
    source: "preset",
    bounds: { minX: -8, minY: -6, width: 16, height: 12 },
    resolution: 0.25,
    objects: [
      { id: "tank-1", type: "device", name: "储罐 01", x: -3.8, y: 2.8, width: 1.7, depth: 1.7, height: 2.1 },
      { id: "tank-2", type: "device", name: "储罐 02", x: -0.8, y: 2.8, width: 1.7, depth: 1.7, height: 2.1 },
      { id: "tank-3", type: "device", name: "储罐 03", x: 2.2, y: 2.8, width: 1.7, depth: 1.7, height: 2.1 },
      { id: "rack-1", type: "device", name: "压缩机组", x: 4.9, y: 0.7, width: 1.8, depth: 2.4, height: 1.3 },
      { id: "crate-1", type: "obstacle", name: "临时物料", x: -2.2, y: -1.1, width: 1.3, depth: 1, height: 0.8 },
      { id: "crate-2", type: "obstacle", name: "施工围挡", x: 1.1, y: -2.1, width: 2.2, depth: 0.55, height: 1.1 },
    ],
    waypoints: [
      { id: 1, name: "设备区入口", x: -6, y: -4, dwell: 2 },
      { id: 2, name: "储罐巡检位", x: -2.2, y: 4.5, dwell: 4 },
      { id: 3, name: "压缩机巡检位", x: 5.4, y: 3.3, dwell: 4 },
      { id: 4, name: "南侧安全点", x: 4.8, y: -4.2, dwell: 2 },
    ],
    createdAt: now,
    updatedAt: now,
  },
];

function hashSeed(seed: string) {
  let hash = 2166136261;
  for (let index = 0; index < seed.length; index += 1) {
    hash ^= seed.charCodeAt(index);
    hash = Math.imul(hash, 16777619);
  }
  return hash >>> 0;
}

function seededRandom(seed: string) {
  let state = hashSeed(seed) || 1;
  return () => {
    state ^= state << 13;
    state ^= state >>> 17;
    state ^= state << 5;
    return (state >>> 0) / 4294967296;
  };
}

export function generatePatrolMap(seedValue: string): PatrolMap {
  const seed = seedValue.trim() || `${Date.now()}`;
  const random = seededRandom(seed);
  const objects: SceneObject[] = [];
  const reserved = [
    { x: -6, y: -4 }, { x: -5, y: 3.9 }, { x: 0, y: 4.2 },
    { x: 5.3, y: 2.4 }, { x: 5.2, y: -3.7 }, { x: 0, y: -4.2 },
  ];
  const count = 8 + Math.floor(random() * 6);
  let attempts = 0;
  while (objects.length < count && attempts < 160) {
    attempts += 1;
    const x = -5.7 + random() * 11.4;
    const y = -3.6 + random() * 7.2;
    const width = 0.65 + random() * 1.5;
    const depth = 0.55 + random() * 1.35;
    if (reserved.some((point) => Math.hypot(point.x - x, point.y - y) < 1.5)) continue;
    if (objects.some((item) => Math.abs(item.x - x) < (item.width + width) / 2 + 0.55 && Math.abs(item.y - y) < (item.depth + depth) / 2 + 0.55)) continue;
    const type: SceneObjectType = objects.length % 3 === 0 ? "device" : "obstacle";
    objects.push({
      id: `${type}-${objects.length + 1}-${hashSeed(`${seed}-${objects.length}`).toString(36)}`,
      type,
      name: type === "device" ? `随机设备 ${objects.length + 1}` : `障碍物 ${objects.length + 1}`,
      x: Number(x.toFixed(2)), y: Number(y.toFixed(2)),
      width: Number(width.toFixed(2)), depth: Number(depth.toFixed(2)),
      height: Number((0.6 + random() * 1.5).toFixed(2)),
    });
  }
  const timestamp = new Date().toISOString();
  return {
    id: `generated-${hashSeed(`${seed}-${timestamp}`).toString(36)}`,
    name: `随机场景 · ${seed.slice(0, 12)}`,
    description: `${objects.length} 个可避障实体 · 种子 ${seed}`,
    source: "generated",
    seed,
    bounds: { minX: -8, minY: -6, width: 16, height: 12 },
    resolution: 0.25,
    objects,
    waypoints: reserved.map((point, index) => ({ id: index + 1, name: `巡检点 ${index + 1}`, ...point, dwell: index === 0 ? 2 : 3 })),
    createdAt: timestamp,
    updatedAt: timestamp,
  };
}

function packBits(values: boolean[]) {
  const bytes = new Uint8Array(Math.ceil(values.length / 8));
  values.forEach((value, index) => {
    if (value) bytes[index >> 3] |= 1 << (index & 7);
  });
  let binary = "";
  const chunk = 0x8000;
  for (let index = 0; index < bytes.length; index += chunk) {
    binary += String.fromCharCode(...bytes.subarray(index, index + chunk));
  }
  return btoa(binary);
}

export function decodeOccupancy(layer?: OccupancyLayer) {
  if (!layer || typeof atob === "undefined") return new Uint8Array();
  const binary = atob(layer.data);
  const cells = new Uint8Array(layer.width * layer.height);
  for (let index = 0; index < cells.length; index += 1) {
    cells[index] = (binary.charCodeAt(index >> 3) >> (index & 7)) & 1;
  }
  return cells;
}

function occupancyBlocksCircle(layer: OccupancyLayer, x: number, y: number, radius: number) {
  const cells = decodeOccupancy(layer);
  if (!cells.length) return false;
  const minColumn = Math.max(0, Math.floor((x - radius - layer.originX) / layer.resolution));
  const maxColumn = Math.min(layer.width - 1, Math.floor((x + radius - layer.originX) / layer.resolution));
  const minMapRow = Math.max(0, Math.floor((y - radius - layer.originY) / layer.resolution));
  const maxMapRow = Math.min(layer.height - 1, Math.floor((y + radius - layer.originY) / layer.resolution));
  for (let mapRow = minMapRow; mapRow <= maxMapRow; mapRow += 1) {
    const cellY = layer.originY + (mapRow + 0.5) * layer.resolution;
    const imageRow = layer.height - 1 - mapRow;
    for (let column = minColumn; column <= maxColumn; column += 1) {
      if (!cells[imageRow * layer.width + column]) continue;
      const cellX = layer.originX + (column + 0.5) * layer.resolution;
      const halfCell = layer.resolution * Math.SQRT1_2;
      if (Math.hypot(cellX - x, cellY - y) <= radius + halfCell) return true;
    }
  }
  return false;
}

export function validatePatrolWaypoints(map: PatrolMap, radius = WAYPOINT_SAFETY_RADIUS): WaypointSafetyIssue[] {
  const maxX = map.bounds.minX + map.bounds.width;
  const maxY = map.bounds.minY + map.bounds.height;
  return map.waypoints.flatMap((waypoint) => {
    if (
      waypoint.x < map.bounds.minX + radius || waypoint.x > maxX - radius ||
      waypoint.y < map.bounds.minY + radius || waypoint.y > maxY - radius
    ) {
      return [{ waypoint, reason: "距离地图边界过近" }];
    }
    const collision = map.objects.find((object) =>
      Math.abs(waypoint.x - object.x) <= object.width / 2 + radius &&
      Math.abs(waypoint.y - object.y) <= object.depth / 2 + radius
    );
    if (collision) return [{ waypoint, reason: `进入“${collision.name}”的安全区` }];
    if (map.occupancy && occupancyBlocksCircle(map.occupancy, waypoint.x, waypoint.y, radius)) {
      return [{ waypoint, reason: "落在栅格障碍物或其安全区内" }];
    }
    return [];
  });
}

export function findNearestSafeWaypointPosition(map: PatrolMap, x: number, y: number) {
  const step = Math.max(map.resolution, 0.2);
  const candidate = (nextX: number, nextY: number) => {
    const waypoint: Waypoint = { id: -1, name: "候选巡检点", x: nextX, y: nextY, dwell: 0 };
    return validatePatrolWaypoints({ ...map, waypoints: [waypoint] }).length === 0;
  };
  if (candidate(x, y)) return { x, y };
  for (let ring = 1; ring <= 40; ring += 1) {
    for (let offset = -ring; offset <= ring; offset += 1) {
      const points = [
        [x + offset * step, y - ring * step], [x + offset * step, y + ring * step],
        [x - ring * step, y + offset * step], [x + ring * step, y + offset * step],
      ];
      for (const [nextX, nextY] of points) {
        if (candidate(nextX, nextY)) return { x: Number(nextX.toFixed(2)), y: Number(nextY.toFixed(2)) };
      }
    }
  }
  return null;
}

function safeNumber(value: unknown, fallback: number, min = -10000, max = 10000) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? Math.max(min, Math.min(max, parsed)) : fallback;
}

function normalizeMap(raw: unknown, fallbackName = "导入地图"): PatrolMap {
  if (!raw || typeof raw !== "object") throw new Error("地图文件内容不是有效对象");
  const input = raw as Partial<PatrolMap>;
  const timestamp = new Date().toISOString();
  const bounds = input.bounds ?? { minX: -8, minY: -6, width: 16, height: 12 };
  const objects = Array.isArray(input.objects) ? input.objects.map((item, index) => ({
    id: String(item.id || `object-${index + 1}`),
    type: item.type === "device" ? "device" as const : "obstacle" as const,
    name: String(item.name || `场景元素 ${index + 1}`),
    x: safeNumber(item.x, 0), y: safeNumber(item.y, 0),
    width: safeNumber(item.width, 1, 0.1, 50), depth: safeNumber(item.depth, 1, 0.1, 50),
    height: safeNumber(item.height, 1, 0.1, 20),
  })) : [];
  const waypoints = Array.isArray(input.waypoints) ? input.waypoints.map((item, index) => ({
    id: safeNumber(item.id, index + 1, 1, 100000),
    name: String(item.name || `巡检点 ${index + 1}`),
    x: safeNumber(item.x, 0), y: safeNumber(item.y, 0), dwell: safeNumber(item.dwell, 3, 0, 3600),
  })) : [];
  return {
    id: `imported-${hashSeed(`${String(input.id || fallbackName)}-${timestamp}`).toString(36)}`,
    name: String(input.name || fallbackName),
    description: String(input.description || "手动导入的地图数据"),
    source: "imported",
    bounds: {
      minX: safeNumber(bounds.minX, -8), minY: safeNumber(bounds.minY, -6),
      width: safeNumber(bounds.width, 16, 2, 500), height: safeNumber(bounds.height, 12, 2, 500),
    },
    resolution: safeNumber(input.resolution, 0.25, 0.02, 5),
    objects,
    waypoints,
    occupancy: input.occupancy,
    createdAt: timestamp,
    updatedAt: timestamp,
  };
}

function yamlValue(text: string, name: string) {
  const match = text.match(new RegExp(`^\\s*${name}\\s*:\\s*(.+?)\\s*$`, "m"));
  return match?.[1]?.replace(/^['"]|['"]$/g, "");
}

function nextPgmToken(bytes: Uint8Array, state: { offset: number }) {
  while (state.offset < bytes.length) {
    const code = bytes[state.offset];
    if (code === 35) {
      while (state.offset < bytes.length && bytes[state.offset] !== 10) state.offset += 1;
    } else if (code <= 32) state.offset += 1;
    else break;
  }
  const start = state.offset;
  while (state.offset < bytes.length && bytes[state.offset] > 32 && bytes[state.offset] !== 35) state.offset += 1;
  return new TextDecoder("ascii").decode(bytes.subarray(start, state.offset));
}

function parsePgm(buffer: ArrayBuffer) {
  const bytes = new Uint8Array(buffer);
  const state = { offset: 0 };
  const magic = nextPgmToken(bytes, state);
  const width = Number(nextPgmToken(bytes, state));
  const height = Number(nextPgmToken(bytes, state));
  const maximum = Number(nextPgmToken(bytes, state));
  if (!(["P2", "P5"].includes(magic)) || !width || !height || !maximum) throw new Error("PGM 文件头无效，仅支持 P2/P5 格式");
  const pixels: number[] = [];
  if (magic === "P2") {
    for (let index = 0; index < width * height; index += 1) pixels.push(Number(nextPgmToken(bytes, state)));
  } else {
    while (state.offset < bytes.length && bytes[state.offset] <= 32) state.offset += 1;
    const twoBytes = maximum > 255;
    for (let index = 0; index < width * height; index += 1) {
      pixels.push(twoBytes ? (bytes[state.offset++] << 8) | bytes[state.offset++] : bytes[state.offset++]);
    }
  }
  if (pixels.length !== width * height || pixels.some((pixel) => !Number.isFinite(pixel))) throw new Error("PGM 像素数量不完整");
  return { width, height, maximum, pixels };
}

export async function importMapFiles(files: File[]) {
  const imported: PatrolMap[] = [];
  const jsonFiles = files.filter((file) => file.name.toLowerCase().endsWith(".json"));
  for (const file of jsonFiles) {
    const raw = JSON.parse(await file.text()) as unknown;
    const candidates = Array.isArray(raw) ? raw : raw && typeof raw === "object" && Array.isArray((raw as { maps?: unknown[] }).maps) ? (raw as { maps: unknown[] }).maps : [raw];
    candidates.forEach((candidate) => imported.push(normalizeMap(candidate, file.name.replace(/\.json$/i, ""))));
  }

  const yamlFiles = files.filter((file) => /\.ya?ml$/i.test(file.name));
  for (const yamlFile of yamlFiles) {
    const yaml = await yamlFile.text();
    const imageName = yamlValue(yaml, "image")?.split(/[\\/]/).pop();
    const pgmFile = files.find((file) => file.name === imageName) ?? files.find((file) => file.name.toLowerCase().endsWith(".pgm"));
    if (!pgmFile) throw new Error(`${yamlFile.name} 缺少配套 PGM 图像，请同时选择 YAML 和 PGM`);
    const pgm = parsePgm(await pgmFile.arrayBuffer());
    const resolution = safeNumber(yamlValue(yaml, "resolution"), 0.05, 0.001, 5);
    const originNumbers = (yamlValue(yaml, "origin")?.match(/-?\d+(?:\.\d+)?/g) ?? ["0", "0"]).map(Number);
    const negate = yamlValue(yaml, "negate") === "1";
    const occupiedThreshold = safeNumber(yamlValue(yaml, "occupied_thresh"), 0.65, 0, 1);
    const occupied = pgm.pixels.map((pixel) => {
      const normalized = pixel / pgm.maximum;
      return (negate ? normalized : 1 - normalized) >= occupiedThreshold;
    });
    const timestamp = new Date().toISOString();
    imported.push({
      id: `ros-map-${hashSeed(`${yamlFile.name}-${timestamp}`).toString(36)}`,
      name: yamlFile.name.replace(/\.ya?ml$/i, ""),
      description: `ROS 2 栅格地图 · ${pgm.width} × ${pgm.height}`,
      source: "imported",
      bounds: { minX: originNumbers[0] || 0, minY: originNumbers[1] || 0, width: pgm.width * resolution, height: pgm.height * resolution },
      resolution,
      objects: [],
      waypoints: [],
      occupancy: { width: pgm.width, height: pgm.height, resolution, originX: originNumbers[0] || 0, originY: originNumbers[1] || 0, data: packBits(occupied) },
      createdAt: timestamp,
      updatedAt: timestamp,
    });
  }
  if (!imported.length) throw new Error("请选择 JSON 地图，或同时选择 ROS YAML 与 PGM 文件");
  return imported;
}

export function mapToRobotPayload(map: PatrolMap) {
  return {
    id: map.id,
    name: map.name,
    bounds: map.bounds,
    resolution: map.resolution,
    objects: map.objects,
    waypoints: map.waypoints,
    occupancy: map.occupancy,
  };
}

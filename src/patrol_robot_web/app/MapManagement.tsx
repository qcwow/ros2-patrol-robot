"use client";

import { useMemo, useRef, useState } from "react";
import { Industrial3DMap, type MapEditorTool } from "./Industrial3DMap";
import { generatePatrolMap, importMapFiles, validatePatrolWaypoints, type PatrolMap, type SceneObject, type Waypoint } from "./mapTypes";

type Props = {
  maps: PatrolMap[];
  activeMapId: string;
  robotX: number;
  robotY: number;
  robotYaw: number;
  onSelect: (map: PatrolMap) => void;
  onApply: (map: PatrolMap) => void;
  onChange: (map: PatrolMap) => void;
  onAdd: (maps: PatrolMap[]) => void;
  onDuplicate: (map: PatrolMap) => void;
  onDelete: (id: string) => void;
  onNotice: (message: string) => void;
  connected: boolean;
  hasPendingChanges: boolean;
  runtimeMap: {
    active_id: string; active_name: string; active_revision?: string | null;
    pending_id?: string | null; pending_name?: string | null;
    source_resolution?: number | null; navigation_resolution?: number | null;
    pending_navigation_resolution?: number | null; resampled?: boolean;
    transitioning: boolean; localization_ready?: boolean;
    error?: string | null; error_map_id?: string | null;
  };
};

const sourceLabels = { preset: "预置", generated: "种子", imported: "导入" } as const;
const toolOptions: Array<{ id: MapEditorTool; icon: string; label: string; help: string }> = [
  { id: "select", icon: "↖", label: "选择移动", help: "点选或拖动物体" },
  { id: "obstacle", icon: "◆", label: "障碍物", help: "在地图上放置障碍" },
  { id: "device", icon: "▣", label: "设备", help: "添加可巡检设备" },
  { id: "waypoint", icon: "⌖", label: "巡检点", help: "添加路线目标点" },
];

export function MapManagement({
  maps, activeMapId, robotX, robotY, robotYaw,
  onSelect, onApply, onChange, onAdd, onDuplicate, onDelete, onNotice,
  connected, hasPendingChanges, runtimeMap,
}: Props) {
  const activeMap = maps.find((map) => map.id === activeMapId) ?? maps[0];
  const [tool, setTool] = useState<MapEditorTool>("select");
  const [zoom, setZoom] = useState(1);
  const [seed, setSeed] = useState("");
  const [selectedEntity, setSelectedEntity] = useState<string | null>(null);
  const [importing, setImporting] = useState(false);
  const [moveStep, setMoveStep] = useState(0.1);
  const fileInput = useRef<HTMLInputElement>(null);

  const selection = useMemo(() => {
    if (!selectedEntity || !activeMap) return null;
    const [kind, id] = selectedEntity.split(":");
    if (kind === "object") {
      const value = activeMap.objects.find((object) => object.id === id);
      return value ? { kind: "object" as const, value } : null;
    }
    const value = activeMap.waypoints.find((waypoint) => waypoint.id === Number(id));
    return value ? { kind: "waypoint" as const, value } : null;
  }, [activeMap, selectedEntity]);

  const safetyIssues = useMemo(
    () => activeMap ? validatePatrolWaypoints(activeMap) : [],
    [activeMap],
  );
  const unsafeWaypointIds = useMemo(
    () => new Set(safetyIssues.map(({ waypoint }) => waypoint.id)),
    [safetyIssues],
  );
  const backendError = runtimeMap.error && runtimeMap.error_map_id === activeMap?.id
    ? runtimeMap.error
    : null;
  const sourceResolution = activeMap.occupancy?.resolution ?? activeMap.resolution;
  const effectiveNavigationResolution = runtimeMap.active_id === activeMap.id && runtimeMap.navigation_resolution
    ? runtimeMap.navigation_resolution
    : Math.min(sourceResolution, 0.1);
  const resolutionWasRefined = effectiveNavigationResolution < sourceResolution - 0.0001;

  if (!activeMap) return null;

  const commit = (patch: Partial<PatrolMap>) => onChange({ ...activeMap, ...patch, updatedAt: new Date().toISOString() });

  const placeEntity = (kind: Exclude<MapEditorTool, "select">, x: number, y: number) => {
    if (kind === "waypoint") {
      const id = Math.max(...activeMap.waypoints.map((point) => point.id), 0) + 1;
      const waypoint: Waypoint = { id, name: `巡检点 ${id}`, x, y, dwell: 3 };
      commit({ waypoints: [...activeMap.waypoints, waypoint] });
      setSelectedEntity(`waypoint:${id}`);
      onNotice(`已添加 ${waypoint.name}`);
      return;
    }
    const index = activeMap.objects.filter((object) => object.type === kind).length + 1;
    const id = `${kind}-${Date.now().toString(36)}`;
    const object: SceneObject = {
      id, type: kind, name: kind === "device" ? `设备 ${index}` : `障碍物 ${index}`,
      x, y, width: kind === "device" ? 1.4 : 1, depth: kind === "device" ? 1 : 0.8,
      height: kind === "device" ? 1.3 : 0.9,
    };
    commit({ objects: [...activeMap.objects, object] });
    setSelectedEntity(`object:${id}`);
    onNotice(`已添加 ${object.name}`);
  };

  const moveEntity = (target: string, x: number, y: number) => {
    const [kind, id] = target.split(":");
    if (kind === "object") commit({ objects: activeMap.objects.map((object) => object.id === id ? { ...object, x, y } : object) });
    else commit({ waypoints: activeMap.waypoints.map((waypoint) => waypoint.id === Number(id) ? { ...waypoint, x, y } : waypoint) });
  };

  const updateObject = (patch: Partial<SceneObject>) => {
    if (!selection || selection.kind !== "object") return;
    commit({ objects: activeMap.objects.map((object) => object.id === selection.value.id ? { ...object, ...patch } : object) });
  };

  const updateWaypoint = (patch: Partial<Waypoint>) => {
    if (!selection || selection.kind !== "waypoint") return;
    commit({ waypoints: activeMap.waypoints.map((waypoint) => waypoint.id === selection.value.id ? { ...waypoint, ...patch } : waypoint) });
  };

  const nudgeSelection = (deltaX: number, deltaY: number) => {
    if (!selection) return;
    const halfWidth = selection.kind === "object" ? selection.value.width / 2 : 0;
    const halfDepth = selection.kind === "object" ? selection.value.depth / 2 : 0;
    const minX = activeMap.bounds.minX + halfWidth;
    const maxX = activeMap.bounds.minX + activeMap.bounds.width - halfWidth;
    const minY = activeMap.bounds.minY + halfDepth;
    const maxY = activeMap.bounds.minY + activeMap.bounds.height - halfDepth;
    const x = Number(Math.max(minX, Math.min(maxX, selection.value.x + deltaX)).toFixed(2));
    const y = Number(Math.max(minY, Math.min(maxY, selection.value.y + deltaY)).toFixed(2));
    if (selection.kind === "object") updateObject({ x, y });
    else updateWaypoint({ x, y });
  };

  const removeSelection = () => {
    if (!selection) return;
    if (selection.kind === "object") commit({ objects: activeMap.objects.filter((object) => object.id !== selection.value.id) });
    else commit({ waypoints: activeMap.waypoints.filter((waypoint) => waypoint.id !== selection.value.id) });
    setSelectedEntity(null);
    onNotice("场景元素已删除");
  };

  const handleImport = async (fileList: FileList | null) => {
    if (!fileList?.length) return;
    setImporting(true);
    try {
      const imported = await importMapFiles(Array.from(fileList));
      onAdd(imported);
      onNotice(`已导入 ${imported.length} 张地图`);
    } catch (error) {
      onNotice(error instanceof Error ? error.message : "地图导入失败");
    } finally {
      setImporting(false);
      if (fileInput.current) fileInput.current.value = "";
    }
  };

  const createRandomMap = () => {
    const next = generatePatrolMap(seed || `${new Date().getTime()}`);
    onAdd([next]);
    setSeed(next.seed ?? "");
    onNotice(`已生成地图：${next.name}`);
  };

  const exportMap = () => {
    const blob = new Blob([JSON.stringify(activeMap, null, 2)], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const anchor = document.createElement("a");
    anchor.href = url;
    anchor.download = `${activeMap.name.replace(/[^\w\u4e00-\u9fa5-]+/g, "-")}.json`;
    anchor.click();
    URL.revokeObjectURL(url);
    onNotice("地图 JSON 已导出");
  };

  const positionNudge = selection ? (
    <div className="position-nudge">
      <div className="position-nudge-head">
        <div><strong>位置微调</strong><small>方向按钮精确移动</small></div>
        <div className="nudge-steps" role="group" aria-label="每次移动距离">
          {[0.1, 0.5, 1].map((step) => <button key={step} className={moveStep === step ? "active" : ""} onClick={() => setMoveStep(step)}>{step} m</button>)}
        </div>
      </div>
      <div className="nudge-pad">
        <button className="nudge-up" aria-label={`向上移动 ${moveStep} 米`} onClick={() => nudgeSelection(0, moveStep)}>↑</button>
        <button className="nudge-left" aria-label={`向左移动 ${moveStep} 米`} onClick={() => nudgeSelection(-moveStep, 0)}>←</button>
        <span>X {selection.value.x.toFixed(2)}　Y {selection.value.y.toFixed(2)}</span>
        <button className="nudge-right" aria-label={`向右移动 ${moveStep} 米`} onClick={() => nudgeSelection(moveStep, 0)}>→</button>
        <button className="nudge-down" aria-label={`向下移动 ${moveStep} 米`} onClick={() => nudgeSelection(0, -moveStep)}>↓</button>
      </div>
    </div>
  ) : null;

  return (
    <section className="map-management" aria-label="地图管理工作台">
      <div className="map-management-head">
        <div>
          <span className="map-kicker">SCENARIO LIBRARY</span>
          <h1>地图管理</h1>
          <p>导入或生成场景，在三维地图中布置障碍物、设备与巡检点。</p>
        </div>
        <div className="map-head-actions">
          <input ref={fileInput} type="file" accept=".json,.yaml,.yml,.pgm" multiple hidden onChange={(event) => void handleImport(event.target.files)} />
          <button className="secondary-map-action" onClick={() => fileInput.current?.click()} disabled={importing}>{importing ? "正在导入…" : "⇧ 导入地图"}</button>
          <button className="secondary-map-action" onClick={exportMap}>⇩ 导出当前地图</button>
          <button className="primary-map-action" onClick={() => onApply(activeMap)} disabled={!connected || runtimeMap.transitioning || safetyIssues.length > 0}>{!connected ? "等待车辆连接" : safetyIssues.length ? `修复 ${safetyIssues.length} 个不安全巡检点后应用` : runtimeMap.transitioning ? runtimeMap.localization_ready === false ? "正在校准车辆定位…" : "正在切换地图…" : runtimeMap.active_id === activeMap.id && !hasPendingChanges && !backendError ? "✓ 已应用到车辆" : runtimeMap.active_id === activeMap.id ? "应用修改到车辆" : "应用到车辆"}</button>
        </div>
      </div>

      <div className="map-workbench">
        <aside className="map-library">
          <div className="map-library-title"><strong>场景库</strong><span>{maps.length} 张地图</span></div>
          <div className="map-card-list">
            {maps.map((map) => (
              <div key={map.id} className={`map-card-row ${map.id === activeMapId ? "active" : ""}`}>
                <button className="map-card" onClick={() => onSelect(map)}>
                  <span className={`map-card-thumb source-${map.source}`}><i></i><b></b><em></em></span>
                  <span className="map-card-copy">
                    <span><em>{sourceLabels[map.source]}</em>{runtimeMap.active_id === map.id && <b>车辆使用中</b>}{runtimeMap.pending_id === map.id && <b>应用中</b>}{map.id === activeMapId && runtimeMap.active_id !== map.id && <b>编辑中</b>}</span>
                    <strong>{map.name}</strong>
                    <small>{map.objects.length} 个实体 · {map.waypoints.length} 个巡检点</small>
                  </span>
                </button>
                <button
                  className="map-card-delete"
                  aria-label={`删除地图${map.name}`}
                  title={runtimeMap.active_id === map.id ? "车辆正在使用，不可删除" : "删除地图"}
                  disabled={maps.length <= 1 || runtimeMap.active_id === map.id}
                  onClick={() => onDelete(map.id)}
                >×</button>
              </div>
            ))}
          </div>
          <div className="seed-generator">
            <div><strong>随机地图种子</strong><small>相同种子会生成相同布局</small></div>
            <input value={seed} onChange={(event) => setSeed(event.target.value)} placeholder="例如 corridor-042" />
            <button onClick={createRandomMap}>✦ 生成测试场景</button>
          </div>
          <div className="map-library-foot">
            <button onClick={() => onDuplicate(activeMap)}>复制</button>
            <button onClick={() => onDelete(activeMap.id)} disabled={maps.length <= 1 || runtimeMap.active_id === activeMap.id}>删除</button>
          </div>
        </aside>

        <div className="map-editor-stage">
          <div className="map-editor-topline">
            <div>
              <input aria-label="地图名称" value={activeMap.name} onChange={(event) => commit({ name: event.target.value })} />
              <span><i className={backendError || safetyIssues.length ? "error" : ""}></i>{backendError ?? (safetyIssues.length ? `请先修复 ${safetyIssues.length} 个标红的不安全巡检点` : `${sourceLabels[activeMap.source]}地图 · ${activeMap.bounds.width.toFixed(1)} × ${activeMap.bounds.height.toFixed(1)} m · ${sourceResolution} m/格${resolutionWasRefined ? ` → Nav2 自动细化为 ${effectiveNavigationResolution.toFixed(2)} m/格` : ""}`)}</span>
            </div>
            <div className="editor-zoom"><button onClick={() => setZoom((value) => Math.max(0.7, value - 0.15))}>−</button><span>{Math.round(zoom * 100)}%</span><button onClick={() => setZoom((value) => Math.min(1.8, value + 0.15))}>＋</button></div>
          </div>
          <div className="map-editor-canvas">
            <Industrial3DMap
              map={activeMap}
              robotX={robotX}
              robotY={robotY}
              robotYaw={robotYaw}
              zoom={zoom}
              selected={0}
              editing
              tool={tool}
              selectedEntity={selectedEntity}
              unsafeWaypointIds={unsafeWaypointIds}
              onSelectEntity={setSelectedEntity}
              onPlace={placeEntity}
              onMoveEntity={moveEntity}
            />
            <div className="editor-toolbox" role="toolbar" aria-label="地图编辑工具">
              {toolOptions.map((option) => <button key={option.id} className={tool === option.id ? "active" : ""} onClick={() => setTool(option.id)} title={option.help}><i>{option.icon}</i><span>{option.label}</span></button>)}
            </div>
            {(safetyIssues.length > 0 || backendError) && (
              <div className="map-safety-alert" role="alert">
                <strong>⚠ 地图无法应用{Boolean(safetyIssues.length) && `：发现 ${safetyIssues.length} 个不安全巡检点`}</strong>
                {safetyIssues.length > 0 ? <ul>
                  {safetyIssues.map(({ waypoint, reason }) => {
                    const routeIndex = activeMap.waypoints.findIndex((item) => item.id === waypoint.id) + 1;
                    return (
                      <li key={waypoint.id}>
                        <button onClick={() => { setSelectedEntity(`waypoint:${waypoint.id}`); setTool("select"); }}>
                          <b>#{routeIndex} {waypoint.name}</b>
                          <span>{reason}</span>
                        </button>
                      </li>
                    );
                  })}
                </ul> : <p>{backendError}</p>}
              </div>
            )}
            <div className="editor-hint">{tool === "select" ? "点选场景元素查看属性，按住并拖动可调整位置" : `在地图空白处点击以添加${toolOptions.find((item) => item.id === tool)?.label}`}</div>
          </div>
        </div>

        <aside className="property-inspector">
          <div className="inspector-heading"><div><span>属性</span><strong>{selection ? selection.value.name : "未选择元素"}</strong></div>{selection && <button aria-label="删除选中元素" onClick={removeSelection}>删除</button>}</div>
          {selection?.kind === "object" ? (
            <div className="inspector-form">
              <label>名称<input value={selection.value.name} onChange={(event) => updateObject({ name: event.target.value })} /></label>
              <label>类型<select value={selection.value.type} onChange={(event) => updateObject({ type: event.target.value as SceneObject["type"] })}><option value="obstacle">障碍物</option><option value="device">设备</option></select></label>
              <div className="inspector-pair"><label>X 坐标<input type="number" step={moveStep} value={selection.value.x} onChange={(event) => updateObject({ x: Number(event.target.value) })} /></label><label>Y 坐标<input type="number" step={moveStep} value={selection.value.y} onChange={(event) => updateObject({ y: Number(event.target.value) })} /></label></div>
              {positionNudge}
              <div className="inspector-section"><span>三维尺寸</span><small>决定导航占用范围和场景高度</small></div>
              <label>宽度 <b>{selection.value.width.toFixed(1)} m</b><input type="range" min="0.2" max="6" step="0.1" value={selection.value.width} onChange={(event) => updateObject({ width: Number(event.target.value) })} /></label>
              <label>深度 <b>{selection.value.depth.toFixed(1)} m</b><input type="range" min="0.2" max="6" step="0.1" value={selection.value.depth} onChange={(event) => updateObject({ depth: Number(event.target.value) })} /></label>
              <label>高度 <b>{selection.value.height.toFixed(1)} m</b><input type="range" min="0.2" max="4" step="0.1" value={selection.value.height} onChange={(event) => updateObject({ height: Number(event.target.value) })} /></label>
            </div>
          ) : selection?.kind === "waypoint" ? (
            <div className="inspector-form">
              <label>巡检点名称<input value={selection.value.name} onChange={(event) => updateWaypoint({ name: event.target.value })} /></label>
              <div className="inspector-pair"><label>X 坐标<input type="number" step={moveStep} value={selection.value.x} onChange={(event) => updateWaypoint({ x: Number(event.target.value) })} /></label><label>Y 坐标<input type="number" step={moveStep} value={selection.value.y} onChange={(event) => updateWaypoint({ y: Number(event.target.value) })} /></label></div>
              {positionNudge}
              <label>到点停留时间<div className="dwell-input"><input type="number" min="0" max="3600" value={selection.value.dwell} onChange={(event) => updateWaypoint({ dwell: Number(event.target.value) })} /><span>秒</span></div></label>
              <div className="waypoint-callout"><i>⌖</i><p><strong>{activeMap.waypoints.findIndex((point) => point.id === selection.value.id) === 0 ? "基地点 · 路线顺序 #1" : `巡检任务 · 路线顺序 #${activeMap.waypoints.findIndex((point) => point.id === selection.value.id) + 1}`}</strong><small>{activeMap.waypoints.findIndex((point) => point.id === selection.value.id) === 0 ? "应用地图时车辆会重置到这里；每轮从此出发并返回。" : "每轮成功到达并完成停留后，剩余次数减 1。"}</small></p></div>
            </div>
          ) : (
            <div className="empty-inspector"><span>↖</span><strong>选择一个场景元素</strong><p>可编辑名称、坐标与三维尺寸；也可以从左侧选择或生成另一张地图。</p></div>
          )}
          <div className="map-summary">
            <span><b>{activeMap.objects.filter((object) => object.type === "obstacle").length}</b>障碍物</span>
            <span><b>{activeMap.objects.filter((object) => object.type === "device").length}</b>设备</span>
            <span><b>{activeMap.waypoints.length}</b>巡检点</span>
          </div>
        </aside>
      </div>
    </section>
  );
}

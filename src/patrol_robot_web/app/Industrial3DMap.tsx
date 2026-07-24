"use client";

import { useEffect, useRef } from "react";
import { decodeOccupancy, waypointType, type PatrolMap } from "./mapTypes";

export type MapEditorTool = "select" | "obstacle" | "device" | "waypoint" | "transit";

type Props = {
  map: PatrolMap;
  robotX: number;
  robotY: number;
  robotYaw: number;
  zoom: number;
  selected: number;
  onSelect?: (id: number) => void;
  editing?: boolean;
  tool?: MapEditorTool;
  selectedEntity?: string | null;
  unsafeWaypointIds?: ReadonlySet<number>;
  onSelectEntity?: (selection: string | null) => void;
  onPlace?: (tool: Exclude<MapEditorTool, "select">, x: number, y: number) => void;
  onMoveEntity?: (selection: string, x: number, y: number) => void;
};

type XY = { x: number; y: number };

export function Industrial3DMap({
  map, robotX, robotY, robotYaw, zoom, selected, onSelect,
  editing = false, tool = "select", selectedEntity = null,
  unsafeWaypointIds, onSelectEntity, onPlace, onMoveEntity,
}: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const dragStateRef = useRef<{
    selection: string;
    pointerId: number;
    offsetX: number;
    offsetY: number;
  } | null>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const context = canvas.getContext("2d");
    if (!context) return;
    let width = 0;
    let height = 0;
    const maxX = map.bounds.minX + map.bounds.width;
    const maxY = map.bounds.minY + map.bounds.height;

    const project = (x: number, y: number): XY => ({
      x: width * (0.08 + ((x - map.bounds.minX) / map.bounds.width) * 0.84),
      y: height * (0.1 + ((maxY - y) / map.bounds.height) * 0.78),
    });
    const unproject = (screenX: number, screenY: number) => ({
      x: map.bounds.minX + ((screenX / width - 0.08) / 0.84) * map.bounds.width,
      y: maxY - ((screenY / height - 0.1) / 0.78) * map.bounds.height,
    });
    const path = (points: XY[], close = false) => {
      context.beginPath();
      points.forEach((point, index) => index === 0 ? context.moveTo(point.x, point.y) : context.lineTo(point.x, point.y));
      if (close) context.closePath();
    };
    const eventPoint = (event: PointerEvent) => {
      const rect = canvas.getBoundingClientRect();
      return {
        x: (event.clientX - rect.left - width / 2) / zoom + width / 2,
        y: (event.clientY - rect.top - height / 2) / zoom + height / 2,
      };
    };
    const hitTest = (screen: XY) => {
      for (let index = map.waypoints.length - 1; index >= 0; index -= 1) {
        const point = project(map.waypoints[index].x, map.waypoints[index].y);
        if (Math.hypot(point.x - screen.x, point.y - screen.y) <= 15) return `waypoint:${map.waypoints[index].id}`;
      }
      for (let index = map.objects.length - 1; index >= 0; index -= 1) {
        const object = map.objects[index];
        const a = project(object.x - object.width / 2, object.y - object.depth / 2);
        const b = project(object.x + object.width / 2, object.y + object.depth / 2);
        const left = Math.min(a.x, b.x) - 10;
        const right = Math.max(a.x, b.x) + 10;
        const top = Math.min(a.y, b.y) - 20 * object.height;
        const bottom = Math.max(a.y, b.y) + 6;
        if (screen.x >= left && screen.x <= right && screen.y >= top && screen.y <= bottom) return `object:${object.id}`;
      }
      return null;
    };
    const entityPosition = (selection: string) => {
      const [kind, id] = selection.split(":");
      if (kind === "object") {
        const object = map.objects.find((item) => item.id === id);
        return object ? { x: object.x, y: object.y } : null;
      }
      const waypoint = map.waypoints.find((item) => item.id === Number(id));
      return waypoint ? { x: waypoint.x, y: waypoint.y } : null;
    };

    const drawOccupancy = () => {
      if (!map.occupancy) return;
      const cells = decodeOccupancy(map.occupancy);
      const bitmap = document.createElement("canvas");
      bitmap.width = map.occupancy.width;
      bitmap.height = map.occupancy.height;
      const bitmapContext = bitmap.getContext("2d");
      if (!bitmapContext) return;
      const pixels = bitmapContext.createImageData(bitmap.width, bitmap.height);
      for (let index = 0; index < cells.length; index += 1) {
        const offset = index * 4;
        pixels.data[offset] = cells[index] ? 87 : 235;
        pixels.data[offset + 1] = cells[index] ? 99 : 237;
        pixels.data[offset + 2] = cells[index] ? 105 : 233;
        pixels.data[offset + 3] = cells[index] ? 225 : 150;
      }
      bitmapContext.putImageData(pixels, 0, 0);
      const topLeft = project(map.occupancy.originX, map.occupancy.originY + map.occupancy.height * map.occupancy.resolution);
      const bottomRight = project(map.occupancy.originX + map.occupancy.width * map.occupancy.resolution, map.occupancy.originY);
      context.imageSmoothingEnabled = false;
      context.drawImage(bitmap, topLeft.x, topLeft.y, bottomRight.x - topLeft.x, bottomRight.y - topLeft.y);
      context.imageSmoothingEnabled = true;
    };

    const draw = () => {
      const rect = canvas.getBoundingClientRect();
      const ratio = Math.min(window.devicePixelRatio || 1, 2);
      canvas.width = Math.max(1, Math.round(rect.width * ratio));
      canvas.height = Math.max(1, Math.round(rect.height * ratio));
      context.setTransform(ratio, 0, 0, ratio, 0, 0);
      width = rect.width;
      height = rect.height;
      context.clearRect(0, 0, width, height);
      context.fillStyle = "#f3f0e8";
      context.fillRect(0, 0, width, height);

      context.save();
      context.translate(width / 2, height / 2);
      context.scale(zoom, zoom);
      context.translate(-width / 2, -height / 2);

      const boundary = [
        project(map.bounds.minX, map.bounds.minY), project(maxX, map.bounds.minY),
        project(maxX, maxY), project(map.bounds.minX, maxY),
      ];
      path(boundary, true);
      context.fillStyle = "#e7e6df";
      context.fill();
      context.strokeStyle = "#aeb7b3";
      context.lineWidth = 2;
      context.stroke();
      drawOccupancy();

      if (!map.occupancy) {
        const insetX = map.bounds.width * 0.08;
        const insetY = map.bounds.height * 0.1;
        const roads = [
          [[map.bounds.minX + insetX, map.bounds.minY + insetY], [maxX - insetX, map.bounds.minY + insetY], [maxX - insetX, maxY - insetY], [map.bounds.minX + insetX, maxY - insetY], [map.bounds.minX + insetX, map.bounds.minY + insetY]],
          [[map.bounds.minX + insetX, map.bounds.minY + map.bounds.height / 2], [maxX - insetX, map.bounds.minY + map.bounds.height / 2]],
        ];
        context.lineCap = "round";
        context.lineJoin = "round";
        roads.forEach((road) => {
          const points = road.map(([x, y]) => project(x, y));
          path(points);
          context.strokeStyle = "#cbd0cf";
          context.lineWidth = 20;
          context.stroke();
          context.strokeStyle = "#faf8f2";
          context.lineWidth = 14;
          context.stroke();
        });
      }

      const drawObject = (object: PatrolMap["objects"][number]) => {
        const base = [
          project(object.x - object.width / 2, object.y - object.depth / 2),
          project(object.x + object.width / 2, object.y - object.depth / 2),
          project(object.x + object.width / 2, object.y + object.depth / 2),
          project(object.x - object.width / 2, object.y + object.depth / 2),
        ];
        const lift = { x: -7 * object.height, y: -15 * object.height };
        const top = base.map((point) => ({ x: point.x + lift.x, y: point.y + lift.y }));
        const active = selectedEntity === `object:${object.id}`;

        path([base[0], base[1], top[1], top[0]], true);
        context.fillStyle = object.type === "obstacle" ? "#a95d49" : "#9daeb5";
        context.fill();
        path([base[1], base[2], top[2], top[1]], true);
        context.fillStyle = object.type === "obstacle" ? "#bf745e" : "#b4c1c5";
        context.fill();
        path(top, true);
        context.fillStyle = active ? "#ffb45c" : object.type === "obstacle" ? "#dd8b72" : "#d9e0e1";
        context.fill();
        context.strokeStyle = active ? "#ef7d14" : object.type === "obstacle" ? "#a34d37" : "#9da9aa";
        context.lineWidth = active ? 3 : 1;
        context.stroke();

        const label = top.reduce((sum, point) => ({ x: sum.x + point.x / 4, y: sum.y + point.y / 4 }), { x: 0, y: 0 });
        context.font = "600 10px Arial, sans-serif";
        context.textAlign = "center";
        context.lineWidth = 4;
        context.strokeStyle = "#ffffffdd";
        context.strokeText(object.name, label.x, label.y + 4);
        context.fillStyle = object.type === "obstacle" ? "#8b4636" : "#52666f";
        context.fillText(object.name, label.x, label.y + 4);
      };
      map.objects.slice().sort((a, b) => b.y - a.y).forEach(drawObject);

      if (map.waypoints.length > 1) {
        path([...map.waypoints, map.waypoints[0]].map((point) => project(point.x, point.y)));
        context.strokeStyle = "#ffffff";
        context.lineWidth = 7;
        context.stroke();
        context.strokeStyle = "#1d78ec";
        context.lineWidth = 4;
        context.stroke();
      }

      map.waypoints.forEach((waypoint, index) => {
        const point = project(waypoint.x, waypoint.y);
        const semanticType = waypointType(waypoint, index);
        const transit = semanticType === "TRANSIT";
        const active = selectedEntity === `waypoint:${waypoint.id}` || (!editing && waypoint.id === selected);
        const unsafe = unsafeWaypointIds?.has(waypoint.id) ?? false;
        if (unsafe) {
          context.beginPath();
          context.arc(point.x, point.y, active ? 17 : 15, 0, Math.PI * 2);
          context.fillStyle = "#ff334433";
          context.fill();
          context.strokeStyle = "#e32636";
          context.lineWidth = 3;
          context.stroke();
        }
        context.beginPath();
        if (transit) {
          const radius = active ? 13 : 11;
          context.moveTo(point.x, point.y - radius);
          context.lineTo(point.x + radius, point.y);
          context.lineTo(point.x, point.y + radius);
          context.lineTo(point.x - radius, point.y);
          context.closePath();
        } else {
          context.arc(point.x, point.y, active ? 12 : 10, 0, Math.PI * 2);
        }
        context.fillStyle = unsafe ? "#e32636" : transit ? "#16a085" : active || semanticType === "HOME" ? "#ff8a24" : "#71899b";
        context.fill();
        context.strokeStyle = unsafe ? "#fff1f2" : "#ffffff";
        context.lineWidth = 3;
        context.stroke();
        context.fillStyle = "#ffffff";
        context.font = "600 10px Arial";
        context.textAlign = "center";
        context.textBaseline = "middle";
        context.fillText(String(index + 1), point.x, point.y + 0.5);
      });

      const robot = project(robotX, robotY);
      const robotFront = project(robotX + Math.cos(robotYaw), robotY + Math.sin(robotYaw));
      const heading = Math.atan2(robotFront.y - robot.y, robotFront.x - robot.x);
      context.globalAlpha = editing ? 0.65 : 1;
      context.beginPath();
      context.arc(robot.x, robot.y, 18, 0, Math.PI * 2);
      context.fillStyle = "#1777e8";
      context.fill();
      context.strokeStyle = "#ffffff";
      context.lineWidth = 4;
      context.stroke();
      context.save();
      context.translate(robot.x, robot.y);
      context.rotate(heading);
      context.beginPath();
      context.moveTo(-5, 7);
      context.lineTo(10, 0);
      context.lineTo(-5, -7);
      context.closePath();
      context.fillStyle = "#ffffff";
      context.fill();
      context.restore();
      context.globalAlpha = 1;
      context.font = "500 11px Arial, sans-serif";
      context.textAlign = "left";
      context.textBaseline = "alphabetic";
      context.lineWidth = 4;
      context.strokeStyle = "#ffffff";
      context.strokeText(editing ? "车辆当前位置" : "巡检车 · 01", robot.x + 24, robot.y + 4);
      context.fillStyle = "#245477";
      context.fillText(editing ? "车辆当前位置" : "巡检车 · 01", robot.x + 24, robot.y + 4);
      context.restore();
    };

    const handlePointerDown = (event: PointerEvent) => {
      const screen = eventPoint(event);
      if (!editing) {
        const hit = hitTest(screen);
        if (hit?.startsWith("waypoint:")) onSelect?.(Number(hit.split(":")[1]));
        return;
      }
      const world = unproject(screen.x, screen.y);
      const boundedX = Math.max(map.bounds.minX, Math.min(maxX, world.x));
      const boundedY = Math.max(map.bounds.minY, Math.min(maxY, world.y));
      if (tool !== "select") {
        onPlace?.(tool, Number(boundedX.toFixed(2)), Number(boundedY.toFixed(2)));
        return;
      }
      const dragSelection = hitTest(screen);
      onSelectEntity?.(dragSelection);
      if (dragSelection) {
        const position = entityPosition(dragSelection);
        dragStateRef.current = {
          selection: dragSelection,
          pointerId: event.pointerId,
          offsetX: position ? position.x - world.x : 0,
          offsetY: position ? position.y - world.y : 0,
        };
        canvas.setPointerCapture(event.pointerId);
        canvas.style.cursor = "grabbing";
      } else {
        dragStateRef.current = null;
      }
    };
    const handlePointerMove = (event: PointerEvent) => {
      const drag = dragStateRef.current;
      if (!drag || drag.pointerId !== event.pointerId || !editing || tool !== "select") return;
      const screen = eventPoint(event);
      const world = unproject(screen.x, screen.y);
      onMoveEntity?.(
        drag.selection,
        Number(Math.max(map.bounds.minX, Math.min(maxX, world.x + drag.offsetX)).toFixed(2)),
        Number(Math.max(map.bounds.minY, Math.min(maxY, world.y + drag.offsetY)).toFixed(2)),
      );
    };
    const endDrag = (event: PointerEvent) => {
      if (dragStateRef.current?.pointerId !== event.pointerId) return;
      dragStateRef.current = null;
      canvas.style.cursor = editing && tool === "select" ? "grab" : "crosshair";
    };

    canvas.style.cursor = dragStateRef.current ? "grabbing" : editing ? tool === "select" ? "grab" : "crosshair" : "default";
    const observer = new ResizeObserver(draw);
    observer.observe(canvas);
    canvas.addEventListener("pointerdown", handlePointerDown);
    canvas.addEventListener("pointermove", handlePointerMove);
    canvas.addEventListener("pointerup", endDrag);
    canvas.addEventListener("pointercancel", endDrag);
    canvas.addEventListener("lostpointercapture", endDrag);
    draw();
    return () => {
      observer.disconnect();
      canvas.removeEventListener("pointerdown", handlePointerDown);
      canvas.removeEventListener("pointermove", handlePointerMove);
      canvas.removeEventListener("pointerup", endDrag);
      canvas.removeEventListener("pointercancel", endDrag);
      canvas.removeEventListener("lostpointercapture", endDrag);
    };
  }, [editing, map, onMoveEntity, onPlace, onSelect, onSelectEntity, robotX, robotY, robotYaw, selected, selectedEntity, tool, unsafeWaypointIds, zoom]);

  return <canvas ref={canvasRef} className="industrial-3d-map" role="img" aria-label={`${map.name} 三维地图，显示设备、障碍物、巡检点、过渡点和车辆位置`} />;
}

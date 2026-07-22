"use client";

import { useEffect, useRef } from "react";
import { decodeOccupancy, type PatrolMap } from "./mapTypes";

type Props = {
  map: PatrolMap;
  robotX: number;
  robotY: number;
  robotYaw: number;
  navigationPath?: XY[];
  zoom: number;
  selected: number;
  onSelect: (id: number) => void;
};

type XY = { x: number; y: number };

export function Industrial2DMap({ map, robotX, robotY, robotYaw, navigationPath = [], zoom, selected, onSelect }: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null);

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
      x: width * (0.055 + ((x - map.bounds.minX) / map.bounds.width) * 0.89),
      y: height * (0.075 + ((maxY - y) / map.bounds.height) * 0.84),
    });
    const line = (points: XY[], close = false) => {
      context.beginPath();
      points.forEach((point, index) => index ? context.lineTo(point.x, point.y) : context.moveTo(point.x, point.y));
      if (close) context.closePath();
    };
    const rectPath = (x: number, y: number, w: number, d: number) => [
      project(x - w / 2, y - d / 2), project(x + w / 2, y - d / 2),
      project(x + w / 2, y + d / 2), project(x - w / 2, y + d / 2),
    ];

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
        pixels.data[offset] = cells[index] ? 73 : 246;
        pixels.data[offset + 1] = cells[index] ? 86 : 248;
        pixels.data[offset + 2] = cells[index] ? 92 : 247;
        pixels.data[offset + 3] = cells[index] ? 225 : 190;
      }
      bitmapContext.putImageData(pixels, 0, 0);
      context.imageSmoothingEnabled = false;
      const topLeft = project(map.occupancy.originX, map.occupancy.originY + map.occupancy.height * map.occupancy.resolution);
      const bottomRight = project(map.occupancy.originX + map.occupancy.width * map.occupancy.resolution, map.occupancy.originY);
      context.drawImage(bitmap, topLeft.x, topLeft.y, bottomRight.x - topLeft.x, bottomRight.y - topLeft.y);
      context.imageSmoothingEnabled = true;
    };

    const draw = () => {
      const rect = canvas.getBoundingClientRect();
      const ratio = Math.min(window.devicePixelRatio || 1, 2);
      width = rect.width;
      height = rect.height;
      canvas.width = Math.max(1, Math.round(width * ratio));
      canvas.height = Math.max(1, Math.round(height * ratio));
      context.setTransform(ratio, 0, 0, ratio, 0, 0);
      context.fillStyle = "#f4f6f5";
      context.fillRect(0, 0, width, height);

      context.save();
      context.translate(width / 2, height / 2);
      context.scale(zoom, zoom);
      context.translate(-width / 2, -height / 2);

      context.strokeStyle = "#dfe4e2";
      context.lineWidth = 0.7;
      const gridSize = map.bounds.width > 60 ? 5 : map.bounds.width > 30 ? 2 : 0.5;
      for (let x = Math.ceil(map.bounds.minX / gridSize) * gridSize; x <= maxX; x += gridSize) {
        line([project(x, map.bounds.minY), project(x, maxY)]);
        context.stroke();
      }
      for (let y = Math.ceil(map.bounds.minY / gridSize) * gridSize; y <= maxY; y += gridSize) {
        line([project(map.bounds.minX, y), project(maxX, y)]);
        context.stroke();
      }

      line([
        project(map.bounds.minX, map.bounds.minY), project(maxX, map.bounds.minY),
        project(maxX, maxY), project(map.bounds.minX, maxY),
      ], true);
      context.fillStyle = "rgba(238, 240, 238, 0.72)";
      context.fill();
      context.strokeStyle = "#8c9997";
      context.lineWidth = 2;
      context.stroke();
      drawOccupancy();

      if (!map.occupancy) {
        const insetX = map.bounds.width * 0.09;
        const insetY = map.bounds.height * 0.12;
        const roads = [
          [[map.bounds.minX + insetX, map.bounds.minY + insetY], [maxX - insetX, map.bounds.minY + insetY], [maxX - insetX, maxY - insetY], [map.bounds.minX + insetX, maxY - insetY], [map.bounds.minX + insetX, map.bounds.minY + insetY]],
          [[map.bounds.minX + insetX, map.bounds.minY + map.bounds.height / 2], [maxX - insetX, map.bounds.minY + map.bounds.height / 2]],
        ];
        context.lineCap = "round";
        context.lineJoin = "round";
        roads.forEach((road) => {
          line(road.map(([x, y]) => project(x, y)));
          context.strokeStyle = "#c7cdcb";
          context.lineWidth = 24;
          context.stroke();
          context.strokeStyle = "#e9ecea";
          context.lineWidth = 18;
          context.stroke();
          context.setLineDash([7, 8]);
          context.strokeStyle = "#b3bcb9";
          context.lineWidth = 1;
          context.stroke();
          context.setLineDash([]);
        });
      }

      map.objects.forEach((object) => {
        const shape = rectPath(object.x, object.y, object.width, object.depth);
        line(shape, true);
        context.fillStyle = object.type === "obstacle" ? "#d98268" : "#d8dddc";
        context.fill();
        context.strokeStyle = object.type === "obstacle" ? "#a84d38" : "#60706e";
        context.lineWidth = 1.5;
        context.stroke();
        const center = project(object.x, object.y);
        context.font = "500 9px Arial, sans-serif";
        context.textAlign = "center";
        context.textBaseline = "middle";
        context.lineWidth = 3;
        context.strokeStyle = "#ffffffd9";
        context.strokeText(object.name, center.x, center.y);
        context.fillStyle = "#465653";
        context.fillText(object.name, center.x, center.y);
      });

      if (map.waypoints.length > 1) {
        line([...map.waypoints, map.waypoints[0]].map((point) => project(point.x, point.y)));
        context.setLineDash([6, 6]);
        context.strokeStyle = "#8fa0ac";
        context.lineWidth = 2;
        context.stroke();
        context.setLineDash([]);
      }
      if (navigationPath.length > 1) {
        line(navigationPath.map((point) => project(point.x, point.y)));
        context.strokeStyle = "#ffffff";
        context.lineWidth = 7;
        context.stroke();
        context.strokeStyle = "#1b75dd";
        context.lineWidth = 4;
        context.stroke();
      }
      map.waypoints.forEach((waypoint, index) => {
        const point = project(waypoint.x, waypoint.y);
        context.beginPath();
        context.arc(point.x, point.y, waypoint.id === selected ? 12 : 10, 0, Math.PI * 2);
        context.fillStyle = waypoint.id === selected || index === 0 ? "#e9822b" : "#637b89";
        context.fill();
        context.strokeStyle = "#ffffff";
        context.lineWidth = 3;
        context.stroke();
        context.fillStyle = "#ffffff";
        context.font = "500 10px Arial";
        context.textAlign = "center";
        context.textBaseline = "middle";
        context.fillText(String(index + 1), point.x, point.y + 0.5);
      });

      const robot = project(robotX, robotY);
      const front = project(robotX + Math.cos(robotYaw), robotY + Math.sin(robotYaw));
      const angle = Math.atan2(front.y - robot.y, front.x - robot.x);
      context.beginPath();
      context.arc(robot.x, robot.y, 17, 0, Math.PI * 2);
      context.fillStyle = "#176fd1";
      context.fill();
      context.strokeStyle = "#ffffff";
      context.lineWidth = 4;
      context.stroke();
      context.save();
      context.translate(robot.x, robot.y);
      context.rotate(angle);
      context.beginPath();
      context.moveTo(-5, 7);
      context.lineTo(10, 0);
      context.lineTo(-5, -7);
      context.closePath();
      context.fillStyle = "#ffffff";
      context.fill();
      context.restore();
      context.font = "500 10px Arial, sans-serif";
      context.textAlign = "left";
      context.textBaseline = "middle";
      context.fillStyle = "#294f67";
      context.fillText(`巡检车 · 01  (${robotX.toFixed(2)}, ${robotY.toFixed(2)})`, robot.x + 23, robot.y);

      const north = project(maxX - map.bounds.width * 0.07, maxY - map.bounds.height * 0.08);
      context.beginPath();
      context.moveTo(north.x, north.y - 14);
      context.lineTo(north.x - 5, north.y + 5);
      context.lineTo(north.x, north.y + 1);
      context.lineTo(north.x + 5, north.y + 5);
      context.closePath();
      context.fillStyle = "#40514f";
      context.fill();
      context.font = "500 9px Arial";
      context.textAlign = "center";
      context.fillText("N", north.x, north.y - 20);
      context.restore();
    };

    const handleClick = (event: MouseEvent) => {
      const rect = canvas.getBoundingClientRect();
      const x = (event.clientX - rect.left - width / 2) / zoom + width / 2;
      const y = (event.clientY - rect.top - height / 2) / zoom + height / 2;
      let best: { id: number; distance: number } | null = null;
      for (const point of map.waypoints) {
        const target = project(point.x, point.y);
        const distance = Math.hypot(target.x - x, target.y - y);
        if (distance < 18 && (!best || distance < best.distance)) best = { id: point.id, distance };
      }
      if (best) onSelect(best.id);
    };

    const observer = new ResizeObserver(draw);
    observer.observe(canvas);
    canvas.addEventListener("click", handleClick);
    draw();
    return () => {
      observer.disconnect();
      canvas.removeEventListener("click", handleClick);
    };
  }, [map, robotX, robotY, robotYaw, navigationPath, selected, zoom, onSelect]);

  return <canvas ref={canvasRef} className="industrial-2d-map" role="img" aria-label={`${map.name} 巡检工程平面图，灰色虚线为巡检点顺序，蓝线为 Nav2 当前实际规划路径`} />;
}

"use client";

import { useEffect, useRef } from "react";

type Point = { id: number; name: string; x: number; y: number };
type Props = {
  robotX: number;
  robotY: number;
  robotYaw: number;
  zoom: number;
  waypoints: Point[];
  selected: number;
  onSelect: (id: number) => void;
};

type XY = { x: number; y: number };

export function Industrial2DMap({ robotX, robotY, robotYaw, zoom, waypoints, selected, onSelect }: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const context = canvas.getContext("2d");
    if (!context) return;
    let width = 0;
    let height = 0;

    const project = (x: number, y: number): XY => ({
      x: width * (0.055 + ((x + 8) / 16) * 0.89),
      y: height * (0.075 + ((6 - y) / 12) * 0.84),
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

      // Fine engineering grid.
      context.strokeStyle = "#dfe4e2";
      context.lineWidth = 0.7;
      for (let x = -8; x <= 8; x += 0.5) {
        line([project(x, -6), project(x, 6)]);
        context.stroke();
      }
      for (let y = -6; y <= 6; y += 0.5) {
        line([project(-8, y), project(8, y)]);
        context.stroke();
      }

      // Site boundary and muted service lanes.
      line(rectPath(0, 0, 15.5, 11.5), true);
    context.fillStyle = "rgba(238, 240, 238, 0.72)";
      context.fill();
      context.strokeStyle = "#8c9997";
      context.lineWidth = 2;
      context.stroke();
      const roads = [
        [[-7, -4.6], [6.8, -4.6], [6.8, 4.6], [-7, 4.6], [-7, -4.6]],
        [[-6.6, 0], [6.3, 0]],
        [[-0.4, -4.5], [-0.4, 4.5]],
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

      const drawEquipment = (name: string, x: number, y: number, w: number, d: number, kind: "rack" | "cabinet" | "room") => {
        const shape = rectPath(x, y, w, d);
        line(shape, true);
        context.fillStyle = kind === "room" ? "#e3e6e5" : "#d8dddc";
        context.fill();
        context.strokeStyle = "#60706e";
        context.lineWidth = 1.5;
        context.stroke();
        const left = Math.min(...shape.map((point) => point.x));
        const right = Math.max(...shape.map((point) => point.x));
        const top = Math.min(...shape.map((point) => point.y));
        const bottom = Math.max(...shape.map((point) => point.y));
        context.strokeStyle = "#899694";
        context.lineWidth = 0.8;
        if (kind === "rack") {
          for (let index = 1; index < 5; index++) {
            const yLine = top + ((bottom - top) * index) / 5;
            line([{ x: left + 3, y: yLine }, { x: right - 3, y: yLine }]);
            context.stroke();
          }
          for (let index = 1; index < 3; index++) {
            const xLine = left + ((right - left) * index) / 3;
            line([{ x: xLine, y: top + 3 }, { x: xLine, y: bottom - 3 }]);
            context.stroke();
          }
        } else {
          context.strokeRect(left + 5, top + 5, Math.max(4, right - left - 10), Math.max(4, bottom - top - 10));
        }
        context.font = "500 10px Arial, sans-serif";
        context.textAlign = "center";
        context.textBaseline = "middle";
        context.fillStyle = "#465653";
        context.fillText(name, (left + right) / 2, (top + bottom) / 2);
      };
      drawEquipment("A区管道架", -2.75, 1.5, 1.0, 4.0, "rack");
      drawEquipment("B区管道架", 2.25, -2.0, 1.0, 4.0, "rack");
      drawEquipment("控制柜", 0.25, 1.0, 1.5, 1.0, "cabinet");
      drawEquipment("北侧设备区", 4.7, 3.7, 2.3, 1.0, "rack");
      drawEquipment("维护间", -5.4, 0.2, 1.7, 1.3, "room");

      // Restrained process lines provide plan detail without a large legend.
      const processLines = [
        { color: "#6f9c9a", points: [[-6.8, 4.9], [6.4, 4.9], [6.4, 1.2], [1.0, 1.2]] },
        { color: "#a58b57", points: [[-3.2, 3.5], [-3.2, -4.2], [2.7, -4.2], [2.7, -0.2]] },
        { color: "#7d8e9a", points: [[-5.8, -0.8], [5.9, -0.8], [5.9, 3.2]] },
      ];
      processLines.forEach((process) => {
        line(process.points.map(([x, y]) => project(x, y)));
        context.strokeStyle = process.color;
        context.lineWidth = 1.5;
        context.stroke();
      });

      // Patrol route.
      if (waypoints.length > 1) {
        line(waypoints.map((point) => project(point.x, point.y)));
        context.strokeStyle = "#ffffff";
        context.lineWidth = 7;
        context.stroke();
        context.strokeStyle = "#1b75dd";
        context.lineWidth = 4;
        context.stroke();
      }
      waypoints.forEach((waypoint, index) => {
        const point = project(waypoint.x, waypoint.y);
        context.beginPath();
        context.arc(point.x, point.y, waypoint.id === selected ? 12 : 10, 0, Math.PI * 2);
        context.fillStyle = waypoint.id === selected ? "#e9822b" : "#637b89";
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

      // Compact north arrow inside the map, no side annotation panel.
      const north = project(6.9, 5.0);
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
      const rawX = event.clientX - rect.left;
      const rawY = event.clientY - rect.top;
      const x = (rawX - width / 2) / zoom + width / 2;
      const y = (rawY - height / 2) / zoom + height / 2;
      let best: { id: number; distance: number } | null = null;
      waypoints.forEach((point) => {
        const target = project(point.x, point.y);
        const distance = Math.hypot(target.x - x, target.y - y);
        if (distance < 18 && (!best || distance < best.distance)) best = { id: point.id, distance };
      });
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
  }, [robotX, robotY, robotYaw, selected, waypoints, zoom, onSelect]);

  return <canvas ref={canvasRef} className="industrial-2d-map" role="img" aria-label="工厂巡检工程平面图，显示设备、管廊、道路、巡检路线和车辆实时位置" />;
}

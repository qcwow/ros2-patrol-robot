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
};

type XY = { x: number; y: number };

const buildings = [
  { name: "A区管道架", x: -2.75, y: 1.5, w: 1.0, d: 4.0, h: 1.2 },
  { name: "B区管道架", x: 2.25, y: -2.0, w: 1.0, d: 4.0, h: 1.2 },
  { name: "控制柜", x: 0.25, y: 1.0, w: 1.5, d: 1.0, h: 1.4 },
  { name: "北侧设备区", x: 4.7, y: 3.7, w: 2.3, d: 1.0, h: 0.8 },
  { name: "维护间", x: -5.4, y: 0.2, w: 1.7, d: 1.3, h: 0.9 },
];

export function Industrial3DMap({ robotX, robotY, robotYaw, zoom, waypoints, selected }: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const context = canvas.getContext("2d");
    if (!context) return;

    const draw = () => {
      const rect = canvas.getBoundingClientRect();
      const ratio = Math.min(window.devicePixelRatio || 1, 2);
      canvas.width = Math.max(1, Math.round(rect.width * ratio));
      canvas.height = Math.max(1, Math.round(rect.height * ratio));
      context.setTransform(ratio, 0, 0, ratio, 0, 0);
      const width = rect.width;
      const height = rect.height;
      context.clearRect(0, 0, width, height);
      context.fillStyle = "#f3f0e8";
      context.fillRect(0, 0, width, height);

      context.save();
      context.translate(width / 2, height / 2);
      context.scale(zoom, zoom);
      context.translate(-width / 2, -height / 2);

      const project = (x: number, y: number): XY => ({
        x: width * (0.08 + ((x + 8) / 16) * 0.84),
        y: height * (0.1 + ((6 - y) / 12) * 0.78),
      });

      const path = (points: XY[], close = false) => {
        context.beginPath();
        points.forEach((point, index) => index === 0 ? context.moveTo(point.x, point.y) : context.lineTo(point.x, point.y));
        if (close) context.closePath();
      };

      // Apple-style pale service roads with a soft neutral curb.
      const roads = [
        [[-7, -4.8], [6.8, -4.8], [6.8, 4.7], [-7, 4.7], [-7, -4.8]],
        [[-6.5, 0], [6.2, 0]],
        [[-0.4, -4.8], [-0.4, 4.7]],
        [[-5.5, 3.1], [4.9, 3.1]],
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

      const drawBuilding = (building: typeof buildings[number]) => {
        const base = [
          project(building.x - building.w / 2, building.y - building.d / 2),
          project(building.x + building.w / 2, building.y - building.d / 2),
          project(building.x + building.w / 2, building.y + building.d / 2),
          project(building.x - building.w / 2, building.y + building.d / 2),
        ];
        const lift = { x: -7 * building.h, y: -15 * building.h };
        const top = base.map((point) => ({ x: point.x + lift.x, y: point.y + lift.y }));

        path([base[0], base[1], top[1], top[0]], true);
        context.fillStyle = "#b6b8ba";
        context.fill();
        path([base[1], base[2], top[2], top[1]], true);
        context.fillStyle = "#c6c7c8";
        context.fill();
        path(top, true);
        context.fillStyle = "#e1dfdd";
        context.fill();
        context.strokeStyle = "#d0cfcd";
        context.lineWidth = 1;
        context.stroke();

        const label = top.reduce((sum, point) => ({ x: sum.x + point.x / 4, y: sum.y + point.y / 4 }), { x: 0, y: 0 });
        context.font = "500 11px Arial, sans-serif";
        context.textAlign = "center";
        context.lineWidth = 4;
        context.strokeStyle = "#ffffffdd";
        context.strokeText(building.name, label.x, label.y + 4);
        context.fillStyle = "#627078";
        context.fillText(building.name, label.x, label.y + 4);
      };
      buildings.slice().sort((a, b) => b.y - a.y).forEach(drawBuilding);

      if (waypoints.length > 1) {
        path(waypoints.map((point) => project(point.x, point.y)));
        context.strokeStyle = "#ffffff";
        context.lineWidth = 7;
        context.stroke();
        context.strokeStyle = "#1d78ec";
        context.lineWidth = 4;
        context.stroke();
      }

      waypoints.forEach((waypoint, index) => {
        const point = project(waypoint.x, waypoint.y);
        context.beginPath();
        context.arc(point.x, point.y, waypoint.id === selected ? 12 : 10, 0, Math.PI * 2);
        context.fillStyle = waypoint.id === selected ? "#ff8a24" : "#71899b";
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
      const robotFront = project(robotX + Math.cos(robotYaw), robotY + Math.sin(robotYaw));
      const heading = Math.atan2(robotFront.y - robot.y, robotFront.x - robot.x);
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
      context.font = "500 11px Arial, sans-serif";
      context.textAlign = "left";
      context.textBaseline = "alphabetic";
      context.lineWidth = 4;
      context.strokeStyle = "#ffffff";
      context.strokeText("巡检车 · 01", robot.x + 24, robot.y + 4);
      context.fillStyle = "#245477";
      context.fillText("巡检车 · 01", robot.x + 24, robot.y + 4);

      context.restore();
    };

    const observer = new ResizeObserver(draw);
    observer.observe(canvas);
    draw();
    return () => observer.disconnect();
  }, [robotX, robotY, robotYaw, selected, waypoints, zoom]);

  return <canvas ref={canvasRef} className="industrial-3d-map" role="img" aria-label="工厂巡检场景三维地图，显示建筑、道路、巡检路线和车辆实时位置" />;
}

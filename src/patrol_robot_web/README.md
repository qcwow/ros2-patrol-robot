# 巡检机器人网页控制台

这是 ROS 2 化工管廊巡检机器人的 Web 前端。页面通过 HTTP 连接
`patrol_robot_web_bridge`，用于显示车辆状态、控制巡检、查看 RGB-D 摄像画面，
以及切换激光雷达、视觉点云和融合避障模式。

## 当前功能

- 车辆连接状态、速度、坐标、朝向和巡检进度
- 2D 工程平面图和 3D 工厂场景
- 巡检启动、停止、急停和路线配置
- 按住方向键人工前进、后退与转向，松开自动停车
- 最高线速度配置和常用速度档位
- RGB-D 实时 JPEG 视频流开启与关闭
- 两轴云台滑动控制、按键微调和一键回中
- 雷达、视觉、融合三种 Nav2 避障模式
- 激光扫描、深度点云、云台和视频流健康状态
- 视觉模式下锁定云台正前方，感知源失效时显示安全停车状态

## 本地运行

要求 Node.js `22.13.0` 或更高版本：

```bash
npm install
npm run dev
```

开发服务器监听 `0.0.0.0:3000`。通过 `robot` 参数指定 Ubuntu 中的 ROS 2 Web
网关地址：

```text
http://虚拟机IP:3000/?robot=http://虚拟机IP:8765
```

网关地址会保存在浏览器本地存储中，后续打开页面时可以继续使用。

## 主要网关接口

| 方法 | 接口 | 用途 |
| --- | --- | --- |
| `GET` | `/api/status` | 获取车辆、巡检、摄像头和感知状态 |
| `GET` | `/api/camera/stream` | 获取低延迟 MJPEG 视频流 |
| `POST` | `/api/patrol/start` | 开始或继续巡检 |
| `POST` | `/api/patrol/stop` | 停止巡检 |
| `POST` | `/api/control/manual` | 发送限幅后的人工速度命令 |
| `POST` | `/api/control/emergency-stop` | 急停车辆 |
| `POST` | `/api/camera/enable` | 开启或关闭网页视频传输 |
| `POST` | `/api/camera/gimbal` | 设置云台水平和俯仰目标 |
| `POST` | `/api/perception/mode` | 切换 `lidar`、`camera` 或 `fusion` 模式 |
| `POST` | `/api/navigation/waypoints` | 更新巡检路线 |
| `POST` | `/api/config/speed` | 更新导航速度限制 |

## 感知模式说明

- `lidar`：只有激光障碍层参与 Nav2 避障。
- `camera`：只有 RGB-D 体素障碍层参与避障，云台锁定正前方。
- `fusion`：激光障碍层和 RGB-D 体素层同时参与避障。

当前模式切换只控制传感器是否参与 Nav2 代价地图，不会给真实硬件断电。视觉模式
已经可以仅使用 RGB-D 点云避障，但 AMCL 定位仍依赖激光 `/scan`。

## 构建与检查

```bash
npm run lint
npm test
```

`npm test` 会生成生产构建，并检查页面可服务、核心控制入口、2D/3D 地图、摄像头
控制和感知模式接口是否存在。

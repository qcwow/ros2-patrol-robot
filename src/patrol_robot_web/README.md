# 巡检机器人网页控制台

这是 ROS 2 化工管廊巡检机器人的 Web 前端。页面通过 HTTP 连接
`patrol_robot_web_bridge`，用于显示车辆状态、控制巡检、查看 RGB-D 摄像画面，
以及切换激光雷达、视觉点云和融合避障模式。

## 实时自主建图

侧栏“自主建图”页面通过 `patrol_robot_web_bridge` 使用端口 `8765` 连接 ROS 2。
页面会实时显示 `/map` 二维栅格、机器人位姿、RGB-D 相机画面和 Frontier
Explorer 状态，并提供按住式方向键、开始建图、一键自主探路、暂停及命名保存。

仿真启动后默认保持人工模式。Frontier Explorer 和 Nav2 服务会提前就绪，但只有
点击网页“一键自主探路”后才会开始自主移动。“放弃本次会话”会清空 SLAM 当前
会话、将统计归零并让仿真车辆返回建图原点。

保存地图时，网关会写入二维导航栅格，并在 OctoMap 数据源可用时同时保存三维
`.ot` 体素文件。新地图会自动同步到侧栏“地图管理”，可继续添加基地点、巡检点
和过渡点后再应用到车辆。

建图仿真现在同时包含静态 `map_server` 和 AMCL。应用自主 SLAM、预置、导入或
随机种子地图时，会在同一进程内从实时 SLAM 安全切换到静态定位，并继续使用原有
Nav2 规划器、控制器、避障策略和巡检管理器；无需重启旧版导航仿真。重新开始建图
时系统会执行反向切换并清空上一轮 SLAM 缓存。巡检路线不连通或通道宽度不足时，
地图会在应用阶段被拒绝，不会等车辆开始巡检后才报告路径失败。

应用一张导入或生成地图后，可以把该场景用于巡检测试，也可以切到“自主建图”
重新扫描它。巡检与建图共享同一车辆和 Nav2 动作服务器，因此使用任务互斥锁：
任一任务运行时，另一任务及地图重新应用按钮都会禁用；停车/暂停或任务完成后才
释放。若暂停建图后选择巡检，网关会自动恢复建图测试前应用的静态地图、AMCL
定位和巡检路线，再启动巡检，无需手工重复应用。

网页与 ROS 运行在不同设备时，可在地址栏指定车辆网关：

```text
http://localhost:3000/?robot=http://Ubuntu虚拟机IP:8765
```

如果虚拟机没有向宿主机开放 `8765` 端口，可在项目根目录建立 SSH 安全通道：

```bash
./scripts/start_web_mapping_tunnel.sh
```

保持该窗口开启，并使用脚本打印的 `127.0.0.1` 网关地址访问控制台。该通道只转发
网页接口，不会启动、停止或修改 Ubuntu 中的仿真。

从 Mac 使用 `run_mapping_vm.sh`、`run_mapping_3d_vm.sh` 或
`run_mapping_3d_gui_vm.sh` 启动集成仿真时，启动脚本会自动建立同一条安全通道，
无需再单独运行 `start_web_mapping_tunnel.sh`。应使用脚本打印的
`127.0.0.1:8765` 网关地址，不再直接连接 VMware 的虚拟机 IP。

地图默认保存到 Ubuntu 的 `~/.ros/patrol_robot/maps`。

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
| `GET` | `/api/mapping/map` | 获取紧凑编码的实时 SLAM 栅格与机器人位姿 |
| `GET` | `/api/mapping/maps` | 获取已保存地图仓库 |
| `POST` | `/api/patrol/start` | 开始或继续巡检 |
| `POST` | `/api/patrol/stop` | 停止巡检 |
| `POST` | `/api/control/manual` | 发送限幅后的人工速度命令 |
| `POST` | `/api/control/emergency-stop` | 急停车辆 |
| `POST` | `/api/camera/enable` | 开启或关闭网页视频传输 |
| `POST` | `/api/camera/gimbal` | 设置云台水平和俯仰目标 |
| `POST` | `/api/perception/mode` | 切换 `lidar`、`camera` 或 `fusion` 模式 |
| `POST` | `/api/navigation/waypoints` | 更新巡检路线 |
| `POST` | `/api/config/speed` | 更新导航速度限制 |
| `POST` | `/api/mapping/start` | 开始手动遥控建图 |
| `POST` | `/api/mapping/explore` | 启动自主前沿探索 |
| `POST` | `/api/mapping/stop` | 停车并暂停建图 |
| `POST` | `/api/mapping/finish` | 停车并按名称保存地图 |

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

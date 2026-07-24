# ROS 2 化工管廊智能巡检机器人

这是一个面向 **Ubuntu 24.04 + ROS 2 Jazzy + Nav2** 的差速巡检机器人项目。
源代码保存在 Mac，ROS 2 编译、仿真和导航运行在 Ubuntu 虚拟机中完成。项目同时
提供不依赖 OpenGL 的轻量二维仿真，以及包含 RGB-D 相机、两轴云台、激光雷达、
未知障碍物和网页控制台的 Gazebo Harmonic 3D 巡检模式。

当前版本已经完成多点自动巡航、网页人工控制、实时摄像画面、RGB-D 点云处理，
以及激光雷达、纯视觉点云和二者融合三种避障感知模式。当前“纯视觉”指不使用
激光障碍层、只使用 RGB-D 深度点云进行避障；AMCL 定位仍依赖激光 `/scan`。
建图模式还可把 RGB-D 点云持续融合为可保存的彩色 OctoMap 三维体素地图。
漏液、漏气、仪表读数等业务识别，以及完全无激光的视觉定位仍属于后续阶段。

## 已包含的功能

- 两轮差速巡检机器人 Xacro 模型
- 2D 激光雷达、IMU、里程计和 TF 仿真
- 可水平、俯仰转动的 RGB-D 云台相机仿真
- RGB 与深度帧同步、相机内参解析和校准投影
- 深度范围过滤、像素抽样、体素降采样和 XYZRGB 点云发布
- 独立 3 Hz 建图点云、彩色 OctoMap 增量融合和 `.ot` 地图保存
- 无 OpenGL 依赖的二维运动与激光仿真
- 简化管廊 Gazebo 场景
- Gazebo 3D 自动循环巡检，以及雷达、RGB-D 视觉或二者融合避障
- Nav2 独立激光与 RGB-D 二维 `ObstacleLayer`
- 感知模式切换时自动停车、更新本地/全局代价地图并清除旧障碍
- 雷达和深度点云健康监控，所选感知源失效时自动停车
- SLAM Toolbox 建图入口
- 预制地图、AMCL 定位和 Nav2 导航
- 基于 YAML 的多点巡航
- 语义路线、逐点精度与限速、有限恢复、BLOCKED/ESTOP 安全状态
- 发车前路径预检，以及 HOME / TRANSIT / INSPECTION 三类路线点
- Nav2 生命周期、传感器、里程计、TF、AMCL、代价地图和车体轮廓健康监控
- 健康异常自动停车、低速续行、安全进度确认和连续故障计数复位
- Web 控制台中的 2D 工程图、3D 场景、实时状态和路线配置
- 地图场景库、JSON / ROS YAML+PGM 导入、可复现随机地图种子和一键切换
- 网页 3D 地图编辑器，可添加、拖动和修改障碍物、设备及巡检点
- 浏览器人工前进、后退、转向、急停和速度限制
- 默认 12 FPS、640 像素宽、只保留最新帧的低延迟 JPEG 视频流
- 网页控制云台水平 `-90°～90°`、向上 `25°`、向下 `35°`
- 雷达、视觉、融合三模式切换和传感器状态显示
- Mac 到虚拟机的一键同步、安装、编译和运行脚本

## 当前系统链路

```text
静态地图 ─────────────────────────────→ 全局代价地图 → NavFn 全局规划

激光雷达 /scan ───────────────→ 激光 ObstacleLayer ─┐
RGB-D 彩色图 + 深度图 → 点云过滤 → 相机 ObstacleLayer ├→ 本地代价地图
                                                     └→ RPP 跟踪、近障减速和碰撞预测

轮速里程计 + IMU → EKF → odom/base_footprint TF
静态地图 + /scan + /odom → AMCL → map/odom TF

建图模式：/scan + /odom → SLAM Toolbox → map/odom TF + 实时二维地图
          RGB-D 点云 + map/camera TF → OctoMap → 彩色三维占据体素

导航健康监控 → 巡检管理器 → 停车 / 有限低速恢复 / BLOCKED

浏览器控制台 ←HTTP 8765→ ROS 2 Web 网关
    ├→ 巡检启停、急停、人工驾驶和速度限制
    ├→ 摄像视频流和两轴云台控制
    └→ 雷达 / 视觉 / 融合感知模式切换
```

### 已完成与后续阶段

| 范围 | 当前状态 |
| --- | --- |
| RGB-D 数据处理 | 已完成同步、深度解析、三维反投影、过滤和 XYZRGB 点云发布 |
| 视觉避障 | 已完成 RGB-D 点云接入 Nav2 本地二维 `ObstacleLayer` |
| 感知切换 | 已完成雷达、视觉、融合三种模式和失效停车保护 |
| 摄像监控 | 已完成网页低延迟画面、开关和两轴云台控制 |
| 视觉定位 | 未完成；当前 AMCL 仍使用激光 `/scan` |
| 长期 3D 地图 | 已完成第一阶段；可增量构建、显示并保存彩色 OctoMap，回环后离线重建待后续实现 |
| 语义 3D 地图 | 未完成；下一阶段把识别掩膜、置信度和时间信息融合到体素或语义地标 |
| Depth Anything | 当前未使用；现阶段优先使用真实 RGB-D 深度，降低车载推理算力需求 |
| 化工业务识别 | 漏液、漏气、烟雾、仪表读数和设备异常识别待后续实现 |

## 项目结构

```text
src/
├── patrol_robot_description/   # 机器人尺寸、URDF/Xacro、传感器
├── patrol_robot_camera/        # RGB-D 同步、深度处理和彩色点云
├── patrol_robot_gazebo/        # Gazebo 场景和 ROS/Gazebo 话题桥接
├── patrol_robot_simulator/     # 默认二维轻量仿真器，不依赖虚拟显卡
├── patrol_robot_navigation/    # 地图、SLAM、AMCL 和 Nav2 参数
├── patrol_robot_patrol/        # 多点巡航管理器和巡航点
├── patrol_robot_web_bridge/    # 浏览器控制台到 ROS 2 的安全 HTTP 网关
├── patrol_robot_web/           # 工厂巡检车辆控制台
└── patrol_robot_bringup/       # 统一启动入口
scripts/                        # Mac 端操作脚本
vm/                             # 虚拟机初始化脚本
```

## 1. 准备虚拟机

虚拟机建议使用 Ubuntu 24.04 Desktop：

- Apple Silicon Mac：Ubuntu 24.04 ARM64
- Intel Mac：Ubuntu 24.04 x86-64
- CPU：6～8 核
- 内存：8～12 GB
- 硬盘：60 GB 以上
- 前端开发：Node.js `22.13.0` 或更高版本
- 3D 图形加速不是必需项；只有启用可选 Gazebo 模式时才需要

Ubuntu 中安装并启动 SSH：

```bash
sudo apt update
sudo apt install openssh-server
sudo systemctl enable --now ssh
hostname -I
```

记下 `hostname -I` 显示的虚拟机 IP。

## 2. 配置 Mac 到虚拟机的连接

在 Mac 项目目录执行：

```bash
cd /Users/qcw/Documents/Ros2
cp .vm.env.example .vm.env
```

编辑 `.vm.env`：

```dotenv
VM_USER=你的Ubuntu用户名
VM_HOST=虚拟机IP
VM_PORT=22
VM_WORKSPACE=/home/你的Ubuntu用户名/robot_patrol_ws
VM_DISPLAY=:0
```

先确认 SSH 可以连接：

```bash
./scripts/vm_shell.sh
```

建议随后配置 SSH 公钥，避免每次同步都输入密码：

```bash
ssh-keygen -t ed25519
ssh-copy-id 用户名@虚拟机IP
```

## 3. 一次性安装 ROS 2 环境

```bash
./scripts/setup_vm.sh
```

该脚本会在 Ubuntu 24.04 中安装 ROS 2 Jazzy、Nav2、Gazebo、SLAM Toolbox、
键盘控制工具和编译依赖。默认巡航不依赖 Gazebo 或 OpenGL。运行时会要求输入
Ubuntu 的 `sudo` 密码。

## 4. 同步并编译

```bash
./scripts/build_vm.sh
```

工作方式是：

1. Mac 内容同步到虚拟机的 `VM_WORKSPACE`。
2. `src` 和配置文件会更新。
3. `build/`、`install/`、`log/` 只保存在虚拟机内部。
4. 虚拟机执行依赖安装和 `colcon build --symlink-install`。

以后每次修改 C++、Python 包结构、Xacro 或构建文件后，重新执行此命令。

## 5. 运行轻量 2D 自动巡航

从 Mac 运行的是无界面模式，适合检查日志和后台巡航：

```bash
./scripts/run_navigation_vm.sh
```

它会启动：

```text
二维仿真器 → /scan + /odom + TF → AMCL → Nav2 → 巡航管理器
```

机器人生成在 `(-6, -4)`，AMCL 使用相同的初始位置。系统等待约 10 秒后开始自动巡航。

SSH 会话没有虚拟机桌面的图形授权，因此这个命令不会打开图形窗口。如需 RViz
导航界面，请在 **Ubuntu 虚拟机桌面** 中打开终端，执行：

```bash
cd ~/robot_patrol_ws
./vm/run_navigation_gui.sh
```

该脚本只打开 RViz，底盘、里程计和激光雷达由轻量二维仿真器生成，因此适合没有 OpenGL 3.3支持的 ARM VMware环境。

轻量二维模式不会生成 RGB-D 图像和深度点云，因此适合验证巡检、地图、AMCL、
Nav2 和激光避障。如果要测试摄像画面、云台或视觉模式，请使用第 7 节的 3D 入口。

## 6. 控制巡航与网页前端

从 Mac 执行：

```bash
./scripts/patrol_control_vm.sh stop
./scripts/patrol_control_vm.sh start
./scripts/patrol_control_vm.sh reset
```

- `stop`：取消当前导航并暂停。
- `start`：从当前巡航点继续。
- `reset`：停止并回到第一个巡航点，随后需要执行 `start`。

### 6.1 使用网页控制台

导航启动文件会同时启动 Web 网关，默认监听 Ubuntu 的 `8765` 端口。先在
Ubuntu 中确认网关正常：

```bash
curl http://127.0.0.1:8765/api/health
```

返回 `{"ok": true, ...}` 即表示网关已经运行。然后在 Ubuntu 查询 IP：

```bash
hostname -I
```

首次使用前，在 Ubuntu 中安装前端依赖：

```bash
cd ~/robot_patrol_ws/src/patrol_robot_web
npm install
```

日常启动前端：

```bash
source ~/.bashrc
cd ~/robot_patrol_ws/src/patrol_robot_web
npm run dev
```

保持这个终端运行。在控制电脑浏览器中打开网页，并通过 `robot` 参数指定小车地址：

```text
http://虚拟机IP:3000/?robot=http://虚拟机IP:8765
```

例如虚拟机地址是 `192.168.64.10`：

```text
http://192.168.64.10:3000/?robot=http://192.168.64.10:8765
```

该地址会保存在当前浏览器中，以后无需重复填写。控制台现在可以：

- 调用 `/patrol_manager/start`、`stop` 和 `reset`。
- 实时读取 `/odom`、`/scan` 与巡检任务状态。
- 在“设备监控”或主视图“摄像画面”中开启、关闭实时画面，并控制两轴云台。
- 在“导航感知模式”中切换雷达、视觉和融合避障，并显示两类传感器健康状态。
- 运行中替换巡检路线，无需重新编译或重启巡检管理器。
- 设置车辆全局速度上限；自动巡检还会叠加每点限速、近障/转弯减速和恢复降速。
- 发送急停命令，并通过 0.5 秒看门狗自动清除失联的手动速度。
- 在“地图管理”中导入、生成、复制和切换测试场景，并把地图与路线同步到 ROS 2。

如果只启动真实底盘和 Nav2，没有使用本项目的统一启动文件，可单独启动网关：

```bash
source install/setup.bash
ros2 run patrol_robot_web_bridge web_bridge
```

防火墙启用时，仅向工厂可信内网放行端口：

```bash
sudo ufw allow from 192.168.1.0/24 to any port 8765 proto tcp
```

网页的“硬件配置”会把配置提交给网关，但车体尺寸、雷达驱动和 TF 必须在
车辆停止后由工程师更新并重启相关节点；它们不会像速度参数一样即时生效。

网页摄像头默认关闭。点击“开启摄像头”后，Web 网关订阅
`/camera/color/image_raw`，以默认 12 FPS、640 像素宽的低延迟 JPEG 视频流
发送给浏览器。编码在独立线程中进行，队列只保留最新一帧，避免视频积压
拖慢导航；关闭后会释放网页视频订阅。摄像头画面与车辆控制共用 `8765`
端口，无需额外开放端口。

画面内的云台控制支持水平 `-90°~90°`、向上 `25°`、向下 `35°`，可滑动、
按键微调或一键回中。Web 网关发布以下弧度制位置命令：

```text
/camera/gimbal/pan/command       水平旋转目标
/camera/gimbal/tilt/command      俯仰旋转目标
```

Gazebo 通过关节位置控制器驱动这两个话题；真机只需让舵机驱动订阅同名话题，
或在启动文件中重映射到实际驱动接口。网页的画面开关只控制视频传输；后台
RGB-D 点云处理和 Nav2 是否使用该点云，由下面的导航感知模式单独控制。

### 6.2 地图管理与场景编辑

左侧进入“地图管理”后，可以在场景库中切换三张预置地图，也可以：

- 输入任意地图种子生成可复现的随机障碍和设备布局；相同种子会得到相同场景。
- 导入本系统导出的 JSON 地图，或一次同时选择标准 ROS 地图的 YAML 与 PGM 文件。
- 在 3D 地图工具栏中选择“障碍物”“设备”或“巡检点”，点击地图完成添加。
- 使用“选择移动”点选并拖动元素，在右侧修改名称、坐标、尺寸、高度和停留时间。
- 导出当前地图为 JSON，便于复用、版本管理和交给其他测试电脑。

地图数据按浏览器本地保存，每张地图独立保留其场景元素和巡检路线。项目 JSON
可以同时保存点位的 `type`、`recovery_policy`、容差和限速；标准 ROS YAML+PGM
只包含占用栅格，导入后仍需补充语义路线。当前编辑器把第一个未标注类型的点解释
为基地，其余未标注点解释为巡检点，不会根据拐角自动生成 `TRANSIT`。

点击场景卡或
“应用到车辆”时，网关会先停车，把场景生成 ROS PGM/YAML 地图，调用
`/map_server/load_map` 切换 Nav2 地图，再清除旧代价地图并同步巡检点。
轻量 2D 仿真器还会同时切换碰撞与激光扫描使用的占用栅格，因此适合直接测试随机
场景。Gazebo 3D 模式会删除上一张网页场景，按照设备与障碍物的宽、深、高实时
生成可视模型和碰撞体，并把机器人重置到新地图的基地点。

### 6.3 切换雷达、视觉和融合避障

控制台提供三种感知模式：

- **雷达模式**：本地代价地图启用激光障碍层；视频仍可单独开启。
- **视觉模式**：本地代价地图仅启用 RGB-D 二维障碍层，云台自动回中并锁定正前方。
- **融合模式**：本地代价地图同时启用两层。

三种模式下全局代价地图都只使用经过审核的静态地图。实时传感器负责局部避障，
不会把临时障碍永久写入全局路线；因此全局路径可能存在，但局部控制器仍可因通道
过窄或实时障碍而拒绝执行。

每次切换前，网关会先停止巡检并发送零速度，等本地代价地图完成参数更新后再清空
旧障碍数据。所选模式要求的感知数据短暂中断时，系统会安全停车，并在数据恢复且
稳定后自动续接原任务；持续异常或底盘/急停故障才要求人工处理。网页按钮切换的是
传感器是否参与 Nav2 避障，
不是给真实硬件断电；真机电源控制应由独立的驱动或安全控制器完成。

也可以直接调用网关：

```bash
curl -X POST http://127.0.0.1:8765/api/perception/mode \
  -H 'Content-Type: application/json' \
  -d '{"mode":"camera"}'
```

> 当前第一阶段完成的是“视觉点云参与避障”。AMCL 定位仍订阅 `/scan`，所以在
> 真机完全关闭激光雷达后，还不能保证长时间自主巡检。下一阶段需要加入视觉里程计
> 和 RGB-D/视觉定位，再将定位源也切换到纯视觉链路。

## 7. 运行 3D 自动巡检、摄像监控与融合避障

先在 Mac 项目目录同步并编译：

```bash
./scripts/build_vm.sh
```

然后在 **Ubuntu 虚拟机桌面终端** 中运行：

```bash
cd ~/robot_patrol_ws
./vm/run_navigation_3d.sh
```

该入口打开 Gazebo 3D 场景，并自动执行：

```text
轮速里程计 + 带噪 IMU → EKF → AMCL → Nav2 → 语义路线巡检
```

为给 Gazebo 和网页控制台留出资源，默认不启动 RViz。需要 RViz 时运行：

```bash
./vm/run_navigation_3d.sh start_rviz:=true
```

RGB-D 相机会同时发布：

```text
/camera/color/image_raw          彩色图
/camera/depth/image_rect_raw     已对齐深度图
/camera/depth/camera_info        深度相机内参
/camera/points/filtered          经过滤的 XYZRGB 点云
/camera/points/mapping           限制为 3 Hz 的持久建图点云
/camera/gimbal/pan/command       云台水平目标（弧度）
/camera/gimbal/tilt/command      云台俯仰目标（弧度）
```

可以在 RViz 中添加 `PointCloud2` 显示，话题选择
`/camera/points/filtered`，Fixed Frame 保持 `map` 或 `odom`。相机数据经过同步、
深度解析、三维反投影和过滤后，会进入 Nav2 本地代价地图的二维障碍层；全局
代价地图仍只使用审核后的静态地图。网页可实时选择是否让视觉参与局部避障。

机器人运行时可通过 `/scan`、`/camera/points/filtered` 或两者共同感知周围设备，
Nav2 障碍层将障碍加入代价地图并规划安全路径；若局部路径无法通行，行为树会
按路线点策略执行有限恢复。短暂健康故障会先停车并清理代价地图，健康连续稳定
1.5 秒后自动低速续接当前目标。恢复后若健康行驶至少 5 秒且向目标前进至少
0.5 m，系统会清零本段连续健康故障记录并恢复当前点正常限速，因此相隔一段
安全路径的异常不会持续累加。健康恢复等待 8 秒内未稳定、同一瓶颈连续完成三次
自动恢复后再次失败，或普通导航重试耗尽后才进入 `BLOCKED`。此时网页的“重新
检查并恢复”始终可点击。急停进入独立
`ESTOP` 锁定，普通启动和任务复位不能解除。
导航健康门会检查 Nav2 生命周期、`/scan`、`/odom`、TF、AMCL 协方差、全局代价
地图新鲜度和机器人轮廓；巡检管理器在每次发车前另行执行目标路径预检。仿真中
低频全局代价地图允许 8 秒更新余量，避免高负载虚拟机把约 3 秒一次的正常发布误判
为超时。真机配置还可强制要求 `/base_driver/ready` 与
`/estop/released`，在底盘看门狗或硬件急停异常时禁止自主巡检。

3D 仿真默认使用更接近真机的 `ekf` 定位链路。需要对照理想真值时可以运行：

```bash
ros2 launch patrol_robot_bringup simulation_navigation.launch.py \
  localization_mode:=ground_truth
```

3D 导航启动后，另开一个 Ubuntu 终端按照第 6.1 节执行 `npm run dev`，即可在
前端查看摄像画面、控制云台，并切换雷达、视觉和融合模式。

## 8. 接入真实 RGB-D 相机

处理器只依赖 ROS 2 标准消息，因此不绑定具体品牌。相机驱动应提供已经完成
深度到彩色对齐的图像，并保证三路消息时间戳接近。启动示例：

```bash
source install/setup.bash
ros2 launch patrol_robot_camera camera_processing.launch.py \
  use_sim_time:=false \
  color_topic:=/你的相机/color/image_raw \
  depth_topic:=/你的相机/aligned_depth_to_color/image_raw \
  camera_info_topic:=/你的相机/aligned_depth_to_color/camera_info
```

如果只接深度相机，可在
`src/patrol_robot_camera/config/rgbd_processor.yaml` 中把 `use_color` 改为
`false`。16 位深度图默认按毫米换算为米；若设备单位不同，需要同步修改
`depth_scale`。安装相机后必须标定 `base_link` 到相机光学坐标系的外参，否则
障碍点云和本地代价地图会出现重影或障碍位置偏移。

虚拟机需要启用 3D 图形加速。若 Gazebo 无法打开或帧率过低，继续使用
`./vm/run_navigation_gui.sh` 可验证相同的导航和避障算法，但显示为 RViz
二维地图。

## 9. 修改巡航点

编辑：

```text
src/patrol_robot_patrol/config/waypoints.yaml
```

格式：

```yaml
frame_id: map
route_id: pipeline_normal_patrol
waypoints:
  - id: home
    name: 基地
    type: HOME
    x: 0.0
    y: 0.0
    yaw: 0.0
    tolerance: {position: 0.08, yaw: 0.10}
    speed_limit: 0.15
    recovery_policy: restricted

  - id: corridor_turn
    name: 通道转弯点
    type: TRANSIT
    x: 1.0
    y: 2.0
    yaw: 1.57
    dwell: 0.0
    tolerance: {position: 0.20, yaw: 0.25}
    speed_limit: 0.30
    recovery_policy: no_spin

  - id: meter_a
    name: 仪表检查点
    type: INSPECTION
    x: 2.0
    y: 2.0
    yaw: 3.14
    dwell: 3.0
    tolerance: {position: 0.08, yaw: 0.10}
    speed_limit: 0.12
    recovery_policy: restricted
    required_sensor: rgbd
```

- `type`：`HOME`、`TRANSIT` 或 `INSPECTION`；路线必须且只能有一个基地。
- `x`、`y`：地图坐标，单位米。
- `yaw`：目标朝向，单位弧度；`1.57` 约为90°，`3.14` 约为180°。
- `dwell`：到达后的停留时间，单位秒。
- `tolerance`：该点实际写入 Nav2 的位置和朝向容差。
- `speed_limit`：前往该点时的最高线速度。
- `recovery_policy`：`standard`、`no_spin` 或 `restricted`。
- `TRANSIT` 默认不计入巡检任务完成数。

旧版仅包含 `name/x/y/yaw/dwell` 的路线仍可加载：首点自动解释为基地，其余点
解释为巡检点。

### 9.1 过渡点与路径预检

`TRANSIT` 是人为定义的语义路线点，用来要求机器人经过通道入口、拐角或转场位置。
Nav2 即使没有过渡点也能沿规划路径转弯，但系统目前不会根据地图拐角自动生成
`TRANSIT`。导入新地图后，应结合车体宽度、通道规则和实地路线手动补充这些点。

每次前往下一点前，巡检管理器先请求一次全局路径作为预检。预检能确认“当前时刻、
当前地图下存在一条候选路线”，但它不会替代正式导航：开始执行后 Nav2 会重新规划，
障碍、定位或代价地图状态也可能变化，因此途中仍可能出现重新规划或找不到路径。

当前默认策略是：

- `HOME`：精确停车，使用 `restricted`，不允许具有明显位移的恢复动作。
- `TRANSIT`：较宽松地经过，使用 `no_spin`，避免在窄通道原地旋转。
- `INSPECTION`：精确对准设备，默认使用 `restricted`。
- `standard`：只用于明确标记的开放区域，可使用更完整的 Nav2 恢复流程。

这些类型和策略来自 `waypoints.yaml`，不是由代价地图自动判断。换地图后需要重新核对
语义点位和恢复策略。

### 9.2 健康监控与失败恢复

系统有两条相互独立但会协作的恢复线路：

1. 导航健康监控持续检查 Nav2 生命周期、传感器数据、`/odom`、TF、AMCL、全局
   代价地图新鲜度和机器人轮廓。短暂异常时立即停车并清理代价地图；健康连续稳定
   1.5 秒后，以降低后的速度继续当前目标。
2. Nav2 正式导航中，局部控制器确认当前路径无法通过后，巡检管理器会从实际全局路径
   和车辆停车位置提取后续完整路线，保留起点、终点安全合流区后写入独立的“失败路径”
   全局代价层。路径预检会计算候选路线与全部失败路线的空间重合度；重合达到 70% 的
   局部平移路线不会发送给控制器，而是继续加入黑名单并重新规划。NavFn/Dijkstra 因而
   会在真正不同的可行路线中选择最短路线。到达当前路线点或人工复位后清除临时记录。
   局部控制器返回“无效路径、无有效控制、未取得进展”等路径可行性错误时，系统不会
   降速重复原路线，而是立即禁用它并规划另一条路线；TF、插件和控制循环超时等基础设施
   故障才使用有界低速重试。最多尝试 6 条不同候选路线，仍无可行替代路线则进入
   `BLOCKED`。单点行为树直接跟随 NavFn 的原始碰撞安全路径，不再让 SimpleSmoother
   原地改写同一变量，保证页面显示路径与控制器接收路径使用同一套几何数据。
   失败路径点云的消息头和代价层备用坐标系也统一为 `map`，避免坐标解释不一致。

健康恢复不是“只要收到一帧全局代价地图就永远健康”，而是要求所有启用的检查项都
持续满足条件。低速恢复后，如果机器人健康行驶至少 5 秒且向目标推进至少 0.5 米，
本段连续健康故障计数会清零；后面再次遇到瓶颈时重新从第一次恢复开始计数。若同一
位置连续完成三次健康恢复后仍再次触发故障，或等待健康恢复超时，则停车进入
`BLOCKED`，由人工确认现场后继续。

### 9.3 速度机制

自动巡检的最终速度由多层限制共同决定：

```text
网页全局速度上限
        ∩
巡检点 speed_limit × 重试/健康恢复降速
        ∩
Nav2 转弯、近障和接近目标自动减速
        ↓
速度平滑器限制加速度和减速度
        ↓
底盘 /cmd_vel
```

路线默认上限约为：基地 `0.15 m/s`、过渡点 `0.35 m/s`、巡检点 `0.15 m/s`。
每增加一次导航重试或健康恢复，当前点速度按 `0.8` 倍递减，但不会低于
`0.06 m/s`。这使机器人在窄路中能够自动低速试探，而离开瓶颈并确认安全进度后恢复
该点的正常限速。Nav2 仍会根据曲率、障碍代价和剩余距离进一步降速，所以这里的数值
都是上限，不是强制速度。

### 9.4 导航回归记录

启动时可以打开指标记录：

```bash
ros2 launch patrol_robot_bringup simulation_navigation.launch.py \
  record_navigation_metrics:=true \
  regression_scenario:=normal_complete
```

结果默认写入 `~/.ros/patrol_robot/navigation_regression/`，包含到达结果、最终误差、
路程、耗时、最小障碍距离、恢复次数和失败原因。场景清单位于
`src/patrol_robot_patrol/config/navigation_regression_scenarios.yaml`。

修改后执行：

```bash
./scripts/build_vm.sh
./scripts/run_navigation_vm.sh
```

## 10. 自己建图

从 Mac 无界面启动建图：

```bash
./scripts/run_mapping_vm.sh
```

或者在 Ubuntu 虚拟机桌面终端中启动带 RViz 窗口的建图：

```bash
cd ~/robot_patrol_ws
./vm/run_mapping_gui.sh
```

另开一个 Mac 终端，用键盘遥控机器人：

```bash
./scripts/teleop_vm.sh
```

完成建图后保存并拉回 Mac：

```bash
./scripts/save_map_vm.sh my_pipeline_map
```

地图会出现在：

```text
src/patrol_robot_navigation/maps/my_pipeline_map.yaml
src/patrol_robot_navigation/maps/my_pipeline_map.pgm
```

运行自建地图时，可以在虚拟机终端指定：

```bash
ros2 launch patrol_robot_bringup simulation_navigation.launch.py \
  map:=/绝对路径/my_pipeline_map.yaml
```

`map` 参数会由统一启动文件传递给 Nav2，因此无需修改源代码即可切换地图。

### 10.1 边定位边建立彩色 3D 体素地图

3D 建图必须使用 Gazebo RGB-D 模式，轻量二维仿真器不生成相机数据。第一次使用
前先同步、安装 OctoMap 依赖并编译：

```bash
./scripts/setup_vm.sh
./scripts/build_vm.sh
```

推荐使用 Ubuntu 图形桌面。该入口会同时打开 Gazebo 和配置好的 RViz，并使用
轮速里程计与 IMU 的 EKF 结果，而不是 Gazebo 真值。Nav2 与前沿探索服务会提前
就绪，但车辆默认保持人工模式；只有点击网页“一键自主探路”后，机器人才会从
实时二维 SLAM 地图中寻找“已知自由区与未知区的边界”，通过 Nav2 逐个接近安全
可达的前沿，并同步扩展彩色 OctoMap：

```bash
cd ~/robot_patrol_ws
./vm/run_mapping_3d_gui.sh
```

如果 VMware 没有可用的 OpenGL 3.3，先登录 Ubuntu 桌面，再从 Mac 用 CPU 软件
渲染启动同一套图形仿真：

```bash
./scripts/run_mapping_3d_gui_vm.sh
```

只有虚拟机具备可用的 EGL / OpenGL 设备时，才使用无界面入口。该脚本会强制
Mesa 软件 OpenGL，以规避 VMware 将不可用虚拟显卡选作 EGL 渲染设备的问题：

```bash
./scripts/run_mapping_3d_vm.sh
```

若日志中 `gazebo` 以 `exit code 139` 退出，说明当前虚拟机的 EGL 无界面渲染仍不
兼容；随后出现的 `controller_server exit code -6` 只是全局关闭时的次生错误。
此时不要修改 Nav2 参数，登录 Ubuntu 图形桌面后改用
`./scripts/run_mapping_3d_gui_vm.sh`。

自主探索会避开占据栅格、拒绝穿越未知区，并将导航失败点暂时加入黑名单。连续
多轮没有前沿时状态为 `COMPLETED`；仅剩不满足安全间距的残留前沿时状态为
`COMPLETED_WITH_UNREACHABLE`，车辆会停车而不会强行接近障碍。

建图启动文件同时运行车辆 Web 网关。按照第 6.1 节启动网页后，点击侧栏
“自主建图”，即可在浏览器中：

- 实时查看 `/map` 逐步展开的二维栅格和 `map -> base_footprint` 机器人位姿
- 查看 RGB-D 相机画面，并用带速度限制和松手看门狗的方向按钮手动扫图
- 一键调用 Frontier Explorer 自主探路，查看前沿数量、到达次数和建图覆盖率
- 输入地图名称，将 YAML + PGM 保存到 `~/.ros/patrol_robot/maps`
- 在地图仓库中删除历史地图，或直接切换到静态定位并部署到机器人内存

建图启动入口同时保留原有静态地图导航能力，不需要为了巡检旧地图再启动另一套
仿真。实时建图时由 SLAM Toolbox 发布定位变换；应用自主建图、预置、导入或随机
种子地图时，系统会先停止车辆，暂停 SLAM，加载静态栅格，再启用 AMCL。两种模式
共用原来的 Nav2 planner、controller、行为树、避障层和巡检管理器。`/map` 由地图
源选择器统一发布，因此不会出现 SLAM 与 map_server 同时抢占地图话题的问题。

“应用地图”还会在切换前检查所有巡检点的车体安全距离和整条路线的连通性。检查
未通过时地图不会生效，巡检按钮也不会放行；运行前 PatrolManager 仍会执行原有的
Nav2 路径预检，作为第二道动态安全检查。

应用导入、随机生成或已保存地图后，可以选择运行巡检，也可以用同一物理场景重新
进行自主建图测试。两类任务互斥：一类运行时另一类入口锁定，只有显式停车/暂停或
任务完成才释放。建图暂停后启动巡检时，系统会自动恢复测试前应用的静态地图和
AMCL 定位；巡检停止后开始建图时，则恢复实时 SLAM 并创建新的建图会话。

从 Mac 使用三个 `run_mapping*_vm.sh` 入口启动时，同一条 SSH 会话会自动把
Ubuntu 的车辆网关转发到 Mac 的 `127.0.0.1:8765`，不依赖 VMware 开放额外端口。
保持启动仿真的终端开启，并使用：

```text
http://localhost:3000/?robot=http%3A%2F%2F127.0.0.1%3A8765
```

若仿真直接在 Ubuntu 桌面终端启动，则可继续使用
`http://Ubuntu虚拟机IP:8765`；也可以在 Mac 另开终端运行
`./scripts/start_web_mapping_tunnel.sh`。

需要手动控制探索流程时，可在 Ubuntu 的另一个终端调用：

```bash
ros2 service call /frontier_explorer/stop std_srvs/srv/Trigger {}
ros2 service call /frontier_explorer/start std_srvs/srv/Trigger {}
ros2 service call /frontier_explorer/reset std_srvs/srv/Trigger {}
ros2 topic echo /frontier_explorer/status
```

如需对照手动建图，先停止自主探索，再缓慢遥控车辆，尽量走出闭环，让 SLAM
Toolbox 能执行回环检测：

```bash
ros2 service call /frontier_explorer/stop std_srvs/srv/Trigger {}
./scripts/teleop_vm.sh
```

RViz 默认同时显示 `/map`、实时 RGB-D 点云和 `/occupied_cells_vis_array` 三维占据
体素。确认已经形成有效体素地图后，在另一个 Mac 终端保存：

```bash
./scripts/save_map_vm.sh my_pipeline_map
./scripts/save_3d_map_vm.sh my_pipeline_map_3d
```

二维导航地图仍保存为 YAML+PGM；彩色三维地图使用能够保留树类型和颜色的 `.ot`
格式，保存位置为：

```text
src/patrol_robot_navigation/maps/3d/my_pipeline_map_3d.ot
```

当前 3D 地图按相机帧到达时的 SLAM 位姿增量融合。若闭环修正幅度很大，已插入
的旧体素不会自动变形，可能看到轻微双墙；后续阶段会记录关键帧并按优化后的轨迹
重建最终地图。移动人员等动态目标也不应写入长期地图，语义阶段会按类别增加衰减。

仓库自带地图由 Gazebo 碰撞几何直接生成，分辨率为 `0.05 m/格`。修改默认世界
中的墙体或设备后应重新生成并校验配套图像：

```bash
python3 scripts/generate_pipeline_map.py
python3 scripts/generate_pipeline_map.py --check
```

## 11. 后续适配真实机器人

当前机器人尺寸集中在：

```text
src/patrol_robot_description/urdf/patrol_robot.urdf.xacro
```

真机阶段重点替换：

- 车体长宽和轮径
- 两轮中心距
- 激光雷达安装位置
- RGB-D 相机与云台安装位置、内参和外参
- Gazebo 差速插件替换为真实底盘驱动节点
- 舵机驱动订阅云台水平和俯仰控制话题
- `/odom`、`/scan`、`/cmd_vel` 和相机话题保持相同接口或通过启动文件重映射

只要上述 ROS 接口保持一致，Nav2、地图和巡航管理器可以继续使用。

## 常用检查命令

进入虚拟机工作空间：

```bash
./scripts/vm_shell.sh
```

然后可以检查：

```bash
ros2 topic list
ros2 topic echo /odom --once
ros2 topic echo /scan --once
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/points/filtered
ros2 topic hz /camera/points/mapping
ros2 topic echo /octomap_binary --once
ros2 action list
ros2 node list
ros2 run tf2_ros tf2_echo odom base_footprint
```

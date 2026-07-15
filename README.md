# ROS 2 管道巡检机器人：自动巡航仿真

这是一个面向 **Ubuntu 24.04 + ROS 2 Jazzy + Nav2** 的差速巡检机器人项目。源代码保存在 Mac，所有 ROS 2 编译、仿真和导航运行都在 Ubuntu 虚拟机中完成。默认使用不依赖 OpenGL 的轻量二维仿真器；同时提供 Gazebo Harmonic 3D 巡检模式，用于展示机器人模型、管廊设备、激光感知和未知障碍物绕行。

当前版本实现自动巡航和第一阶段 RGB-D 相机处理，不包含漏液、漏气等业务检测。

## 已包含的功能

- 两轮差速巡检机器人 Xacro 模型
- 2D 激光雷达、IMU、里程计和 TF 仿真
- 前向 RGB-D 相机仿真、彩色/深度帧同步和校准投影
- 深度范围过滤、像素抽样、体素降采样和 XYZRGB 点云发布
- 无 OpenGL依赖的二维运动与激光仿真
- 简化管廊 Gazebo 场景
- Gazebo 3D 自动循环巡检与激光雷达避障
- SLAM Toolbox 建图入口
- 预制地图、AMCL 定位和 Nav2 导航
- 基于 YAML 的多点巡航
- 到点停留、循环巡航、导航超时、自动重试和失败跳过
- Mac 到虚拟机的一键同步、安装、编译和运行脚本

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
- 硬盘：60 GB以上
- 3D 图形加速不是必需项；只有启用可选 Gazebo模式时才需要

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

该脚本会在 Ubuntu 24.04 中安装 ROS 2 Jazzy、Nav2、Gazebo、SLAM Toolbox、键盘控制工具和编译依赖。默认巡航不依赖 Gazebo或 OpenGL。运行时会要求输入 Ubuntu 的 `sudo` 密码。

## 4. 同步并编译

```bash
./scripts/build_vm.sh
```

工作方式是：

1. Mac 内容同步到虚拟机的 `VM_WORKSPACE`。
2. `src` 和配置文件会更新。
3. `build/`、`install/`、`log/` 只保存在虚拟机内部。
4. 虚拟机执行依赖安装和 `colcon build --symlink-install`。

以后每次修改 C++、Python包结构、Xacro或构建文件后，重新执行此命令。

## 5. 直接运行自动巡航

从 Mac 运行的是无界面模式，适合检查日志和后台巡航：

```bash
./scripts/run_navigation_vm.sh
```

它会启动：

```text
二维仿真器 → /scan + /odom + TF → AMCL → Nav2 → 巡航管理器
```

机器人生成在 `(-6, -4)`，AMCL 使用相同的初始位置。系统等待约10秒后开始自动巡航。

SSH 会话没有虚拟机桌面的图形授权，因此这个命令不会打开图形窗口。如需 RViz导航界面，请在 **Ubuntu 虚拟机桌面** 中打开终端，执行：

```bash
cd ~/robot_patrol_ws
./vm/run_navigation_gui.sh
```

该脚本只打开 RViz，底盘、里程计和激光雷达由轻量二维仿真器生成，因此适合没有 OpenGL 3.3支持的 ARM VMware环境。

## 6. 控制巡航

从 Mac 执行：

```bash
./scripts/patrol_control_vm.sh stop
./scripts/patrol_control_vm.sh start
./scripts/patrol_control_vm.sh reset
```

- `stop`：取消当前导航并暂停。
- `start`：从当前巡航点继续。
- `reset`：停止并回到第一个巡航点，随后需要执行 `start`。

## 6.1 使用网页控制台

导航启动文件会同时启动 Web 网关，默认监听 Ubuntu 的 `8765` 端口。先在
Ubuntu 中确认网关正常：

```bash
curl http://127.0.0.1:8765/api/health
```

返回 `{"ok": true, ...}` 即表示网关已经运行。然后在 Ubuntu 查询 IP：

```bash
hostname -I
```

在控制电脑浏览器中打开网页，并通过 `robot` 参数指定小车地址：

```text
http://控制台地址/?robot=http://192.168.1.50:8765
```

该地址会保存在当前浏览器中，以后无需重复填写。控制台现在可以：

- 调用 `/patrol_manager/start`、`stop` 和 `reset`。
- 实时读取 `/odom`、`/scan` 与巡检任务状态。
- 在“设备监控”或主视图“摄像画面”中开启、关闭实时画面，并控制两轴云台。
- 运行中替换巡检路线，无需重新编译或重启巡检管理器。
- 动态修改 Nav2 控制器和速度平滑器的最大速度。
- 发送急停命令，并通过 0.5 秒看门狗自动清除失联的手动速度。

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
或在启动文件中重映射到实际驱动接口。网页开关只控制实时画面，不会停止后台
RGB-D 点云处理，也不会自动把相机接入 Nav2。

## 6.1 运行 3D 自动巡检与避障

先在 Mac 项目目录同步并编译：

```bash
./scripts/build_vm.sh
```

然后在 **Ubuntu 虚拟机桌面终端** 中运行：

```bash
cd ~/robot_patrol_ws
./vm/run_navigation_3d.sh
```

该入口同时打开 Gazebo 3D 场景和 RViz，并自动执行：

```text
激光雷达感知 → AMCL 定位 → Nav2 路径规划/局部避障 → 多点循环巡检
```

RGB-D 相机会同时发布：

```text
/camera/color/image_raw          彩色图
/camera/depth/image_rect_raw     已对齐深度图
/camera/depth/camera_info        深度相机内参
/camera/points/filtered          经过滤的 XYZRGB 点云
/camera/gimbal/pan/command       云台水平目标（弧度）
/camera/gimbal/tilt/command      云台俯仰目标（弧度）
```

可以在 RViz 中添加 `PointCloud2` 显示，话题选择
`/camera/points/filtered`，Fixed Frame 保持 `map` 或 `odom`。当前阶段已经完成
相机数据同步、深度解析、三维反投影与点云过滤；体素地图累积和将相机障碍接入
Nav2 属于下一阶段。

机器人运行时通过 `/scan` 感知周围设备，Nav2 障碍层将障碍加入代价
地图并规划安全路径；若局部路径无法通行，行为树会触发等待、后退/旋转
和重新规划。

## 6.2 接入真实 RGB-D 相机

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
后续体素地图会出现重影或障碍位置偏移。

虚拟机需要启用 3D 图形加速。若 Gazebo 无法打开或帧率过低，继续使用
`./vm/run_navigation_gui.sh` 可验证相同的导航和避障算法，但显示为 RViz
二维地图。

## 7. 修改巡航点

编辑：

```text
src/patrol_robot_patrol/config/waypoints.yaml
```

格式：

```yaml
frame_id: map
waypoints:
  - name: 检查点A
    x: 1.0
    y: 2.0
    yaw: 1.57
    dwell: 3.0
```

- `x`、`y`：地图坐标，单位米。
- `yaw`：目标朝向，单位弧度；`1.57` 约为90°，`3.14` 约为180°。
- `dwell`：到达后的停留时间，单位秒。

修改后执行：

```bash
./scripts/build_vm.sh
./scripts/run_navigation_vm.sh
```

## 8. 自己建图

从 Mac 无界面启动建图：

```bash
./scripts/run_mapping_vm.sh
```

或者在 Ubuntu虚拟机桌面终端中启动带 RViz窗口的建图：

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

## 9. 后续适配真实机器人

当前机器人尺寸集中在：

```text
src/patrol_robot_description/urdf/patrol_robot.urdf.xacro
```

真机阶段重点替换：

- 车体长宽和轮径
- 两轮中心距
- 激光雷达安装位置
- Gazebo差速插件替换为真实底盘驱动节点
- `/odom`、`/scan`、`/cmd_vel` 保持相同接口

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
ros2 action list
ros2 node list
ros2 run tf2_ros tf2_echo odom base_footprint
```

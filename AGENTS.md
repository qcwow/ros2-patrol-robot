# AGENTS.md

本文件适用于 `/Users/qcw/Documents/Ros2` 及其全部子目录。

## 项目定位

这是一个基于 Ubuntu 22.04、ROS 2 Humble、Nav2、AMCL 和 SLAM Toolbox 的化工
管廊巡检机器人项目。代码需要同时支持：

- Mac 上编辑，Ubuntu Humble 虚拟机中进行轻量 2D/Gazebo 3D 仿真；
- 作为 overlay 运行在 ROSOrin Orin NX 麦轮真车上；
- 语义巡检路线、网页人工控制、RGB-D 感知、二维/三维建图；
- 对人工驾驶和自主导航都提供失联停车与碰撞安全保护。

`/Users/qcw/Documents/Ros2` 是本项目的唯一主工作区。厂家 ROSOrin 代码默认只用于
接口对照；除非用户明确要求，不要直接修改厂家源码，也不要把修改只留在
`/Users/qcw/src` 或其他代码镜像中。

## 目录职责

- `src/patrol_robot_bringup`：仿真和真车的顶层启动入口。
- `src/patrol_robot_navigation`：SLAM、AMCL、Nav2、地图和真车 profile。
- `src/patrol_robot_patrol`：巡检状态机、路线模型、健康门控和速度安全节点。
- `src/patrol_robot_web_bridge`：网页控制、Nav2/人工速度仲裁和遥测。
- `src/patrol_robot_simulator`：无 OpenGL 的轻量 2D 仿真器。
- `src/patrol_robot_gazebo`：Gazebo 场景与桥接。
- `src/patrol_robot_camera`：RGB-D 图像、深度和点云处理。
- `src/patrol_robot_description`：仿真机器人 URDF/Xacro。
- `scripts`：Mac、虚拟机和真车命令入口。
- `vm`：Ubuntu 虚拟机桌面环境中的运行脚本。

不要编辑或提交 `build/`、`install/`、`log/`、`node_modules/`、`.next/`、`dist/`
等生成目录。不要读取、打印或提交 `.vm.env`、SSH 密钥和其他凭据。

## 支持矩阵与接口

| 环境 | ROS 版本 | 雷达 | 最终底盘速度 | `use_sim_time` |
| --- | --- | --- | --- | --- |
| 2D/Gazebo 仿真 | Humble | `/scan` | `/cmd_vel` | `true` |
| ROSOrin 真车 | Humble | `/scan_raw` | `/controller/cmd_vel` | `false` |

仿真参数和真车参数必须分离：

- 仿真：`nav2_params.yaml`、`slam_toolbox.yaml`；
- 真车：`nav2_params_real_car.yaml`、`slam_toolbox_real_car.yaml`、
  `manual_lidar_safety_real_car.yaml`。

不要为了真车而直接改坏仿真默认值，也不要在业务节点中硬编码
`/home/ubuntu/ros2_ws` 等机器路径。话题、文件和设备路径应通过 launch 参数或 ROS
参数传入。

真车复用厂家 `controller`、`peripherals`、`slam` 包。启动前检查这些包存在；不要
在本项目中复制同名厂家包。

## 真车速度安全链

真车速度必须遵守以下唯一链路：

```text
手柄 / 网页人工控制 / Nav2
        ↓
原始速度话题
        ↓
manual_lidar_safety + /scan_raw
        ↓
/cmd_vel_safety_checked
        ↓
base_command_watchdog
        ↓
/controller/cmd_vel
```

强制规则：

- 除 `base_command_watchdog` 外，本项目节点不得向真车
  `/controller/cmd_vel` 直接发布。
- 真车模式下，Web Bridge 应发布到 `/cmd_vel_base_raw`，不能发布到物理底盘话题。
- 手柄和键盘遥控应发布到 `/cmd_vel_manual_raw`，不能绕过雷达过滤。
- 雷达数据过期、速度源过期、节点退出或输入格式异常时必须输出零速。
- 看门狗和碰撞过滤使用单调时间或系统墙钟，不能依赖可能暂停的仿真 `/clock`。
- 不要无测试地缩小车体轮廓、清障距离、制动距离或超时时间。

当前真车安全轮廓以厂家 CAD 为依据：前向约 `0.16 m`、后向约 `0.15 m`、半宽约
`0.13 m`。这不是允许贴墙行驶的尺寸；还需保留 footprint padding、硬安全余量和
动态制动距离。

二维激光雷达不能可靠识别玻璃、低矮障碍和高于扫描面的障碍。软件急停和巡检
`ESTOP` 是逻辑锁存，不等同于硬件急停。不要声称软件能够保证绝不碰撞。

发现 TF 冲突、定位跳变、地图不匹配、雷达陈旧、轮廓错误或安全节点缺失时，禁止
继续运动测试。

## ROS 2 与 Nav2 约束

- 目标版本固定为 ROS 2 Humble；不要引入只在 Jazzy/Rolling 存在的 API。
- `map -> odom` 同一时刻只能有一个发布者：建图时由 SLAM Toolbox 发布，静态地图
  导航时由 AMCL 发布。
- 真车 AMCL 不得默认把初始位姿设为 `(0, 0)`；保持 `set_initial_pose: false`，等待
  操作者在 RViz 确认 `2D Pose Estimate`。
- Nav2 controller、behavior 和恢复速度先进入 `cmd_vel_nav_raw`，再经
  `velocity_smoother` 输出；不得新增直通物理底盘的恢复动作。
- 初次实车测试禁止盲目旋转、后退等位移恢复。恢复次数必须有界，耗尽后进入
  `BLOCKED`，等待人工确认。
- 传感器 QoS 优先使用 `qos_profile_sensor_data`；锁存状态使用可靠、
  `TRANSIENT_LOCAL` QoS。
- 回调中不要执行长时间 sleep、同步等待 action/service 或阻塞网络 I/O。
- 安全、几何和路线判断尽量放入不依赖 ROS 的纯函数，并为其添加单元测试。
- 新增 Python 节点时同步更新 `setup.py` console entry point 和 `package.xml` 依赖。
- 新增 launch/config/map 时确认 CMake 或 setuptools 会安装该文件。

## 巡检业务不变量

- 路线必须且只能包含一个显式 `HOME`。
- `TRANSIT` 只约束通行路线，不计为巡检任务。
- `INSPECTION` 才能完成巡检计数，并可声明停留时间、容差、限速和必需传感器。
- 发车前必须通过导航健康门和路径预检。
- `required_sensor` 只能使用路线模型支持的传感器名称。
- 到达、失败、恢复和任务计数必须由 action 结果确认，不能仅靠固定时间推测。
- 基础设施故障不得错误写入失败路线黑名单；只有经过验证的路径/控制可行性失败才
  能排除候选路线。
- `ESTOP` 不能通过普通 start/reset 自动解除。

真车默认使用 `waypoints_real_car_template.yaml`，且 `patrol_autostart` 必须保持
`false`，直到地图坐标与全部 HOME/TRANSIT/INSPECTION 点经过现场核对。

## Web Bridge 约束

- 仿真默认 `base_command_topic=/cmd_vel`、`scan_topic=/scan`。
- 真车必须设置 `base_command_topic=/cmd_vel_base_raw`、
  `scan_topic=/scan_raw`。
- 人工控制接管时暂停巡检/探索，并保持手动 latch，直到目标取消完成。
- 手动命令必须有限速和超时停车；零速不能立刻释放 latch 而让陈旧 Nav2 指令恢复。
- 图像编码、HTTP 和文件操作不得阻塞 ROS 控制回调。

## 修改原则

- 工作区经常包含用户尚未提交的修改。先执行 `git status --short`，保留所有无关
  改动；不要 reset、checkout、覆盖或删除用户文件。
- 优先做最小、可回滚的修改。不要顺手重构与任务无关的大文件。
- 不删除 ZIP、输出、地图、文档等未跟踪文件，除非用户明确指定目标。
- 不提交、push 或发布代码，除非用户明确要求。
- 变更安全参数时，在交付说明中列出旧值、新值、依据和验证结果。
- 如果真车/虚拟机离线，完成本地可做的验证并明确标记未完成的运行时检查，不得把
  静态测试描述为实车通过。

## 交付检查清单

完成修改前至少确认：

- 改动位于 `/Users/qcw/Documents/Ros2`，没有只改厂家镜像；
- 仿真与真车 profile 没有串用 `/scan`、`/scan_raw` 或 `use_sim_time`；
- 真车所有速度发布者均经过雷达过滤和最终看门狗；
- 建图与 AMCL 没有同时发布 `map -> odom`；
- 巡检默认不会因构建或 launch 自动发车；
- Python、YAML、XML、Shell 和相关单元测试已验证；
- 未执行的 VM、真车构建或运动测试已明确说明；
- 交付信息包含关键文件、验证结果和安全限制。

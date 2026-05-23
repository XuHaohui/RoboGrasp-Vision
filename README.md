# RoboGrasp-Vision

**视觉驱动自主抓取研究平台**

RoboGrasp-Vision 是一个**以 C++ 为主**的模块化机器人视觉抓取系统，在 [RoboGrasp-Pipeline (piper_control)](../piper_conrtrol) 的底层操作能力之上，增加感知层与自主决策层，让机器人能够"看见物体 → 估计位置 → 生成抓取指令 → 调用底层执行"。

主要开发语言：**C++ (rclcpp + ament_cmake)**，launch 文件使用 Python。

```
+------------------+                         +------------------+
|   Perception     | ──────────────────────> |   Robot Bridge   |
| (object detect,  |                         | (command relay)  |
|  3D localization)|                         |                  |
+------------------+                         +--------+---------+
        |                                               |
        v                                               v
+------------------------+                +------------------------+
|   Sensor Input (mock   |                | RoboGrasp-Pipeline     |
|   or real RGB-D)       |                | (piper_control FSM)    |
+------------------------+                +------------------------+
```

## 目录结构

```
RoboGrasp-Vision/
├── README.md                          # 本文件
├── .gitignore
├── scripts/
│   ├── setup_env.sh                   # 环境加载脚本
│   └── mvp_interactive.sh             # 一键键盘交互启动脚本
└── src/
    ├── robograsp_interfaces/          # 自定义 ROS2 消息/服务接口
    │   ├── msg/
    │       │   ├── PerceptionResult.msg   # 感知结果
    │   │   └── ObjectInfo.msg         # 物体信息（桥接输出）
    │
    ├── robograsp_vision_perception/   # 感知层
    │   ├── mock_perception_node.cpp   # Mock 感知节点（键盘/参数触发）
    │   ├── mock_detector.cpp          # Mock 检测器（集中管理物体数据）
    │   ├── perception_node.cpp        # 主感知节点
    │   ├── detector.hpp               # 检测器抽象基类
    │   └── config/
    │       └── perception_params.yaml
    │
    ├── robograsp_vision_bridge/       # 机器人桥接层
    │   ├── bridge_node.cpp            # 桥接节点
    │   ├── piper_control_interface.cpp # Piper 控制接口
    │   ├── robot_interface.hpp        # 机器人接口抽象
    │   └── config/
    │       └── bridge_params.yaml
    │
    └── robograsp_vision_bringup/      # 启动与配置
        ├── launch/
        │   ├── demo.launch.py         # 完整启动文件
        │   └── mvp_demo.launch.py     # MVP 最小启动
        └── config/
            ├── system_params.yaml     # 系统级参数
            ├── camera_params.yaml     # 相机参数预设
            └── object_classes.yaml    # 物体类别配置
```

## 系统架构

### 数据流 (MVP)

```
[键盘/param]                          [bridge_node]                     [piper_control]
     │                                     │                                  │
     ▼                                     ▼                                  │
[mock_perception]  ──PerceptionResult──►  tf2变换 + ObjectInfo  ──►  /target_pose
     │                                     │                                  │
     │  ObjectDetector                    │  PiperControlInterface           │
     │  └─ MockDetector                   │  └─ send_object_info()           │
     │     └─ objects_                    │     └─ publish(ObjectInfo)       │
     │        {name, x,y,z,               │                                  │
     │         bbox, shape,                │                                  │
     │         confidence}                 ▼                                  │
     ▼                              ┌─────────────┐                    ┌──────▼──────┐
  /perception/result                │  bridge_node │                    │ MoveIt2 FSM │
                                    └─────────────┘                    │ (Pick&Place)│
                                                                       └─────────────┘
```

### 模块职责

| 模块 | 职责 | 输入 | 输出 |
|------|------|------|------|
| **perception** | 物体检测、3D 定位 | 键盘/参数触发 (或相机) | `PerceptionResult` |
| **bridge** | 坐标变换、指令转发、后端适配 | `PerceptionResult` | `/target_pose` (ObjectInfo) |
| **bringup** | 统一启动入口、参数管理 | — | launch 文件 |

### 关键设计决策

- **接口优先**：各层通过自定义 ROS2 消息松耦合，可独立替换
- **抽象基类**：detector、robot_interface 均为 ABC，支持多后端
- **Mock 先行**：MVP 阶段全 mock 数据，零硬件依赖即可运行完整数据流
- **C++ 为主**：感知、桥接层均使用 C++ (rclcpp)，与 piper_control 语言栈一致

## 快速开始

### 环境要求

- Ubuntu 22.04
- ROS 2 Humble (安装于 `/opt/ros/humble`)
- Python 3.10+

### 构建

```bash
cd RoboGrasp-Vision
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source scripts/setup_env.sh
```

### 运行 MVP Demo

```bash
# 一键键盘交互启动（推荐）
# 启动后按 1/2/3 触发对应物体的模拟检测，按 q 退出
./scripts/mvp_interactive.sh
```

菜单示例：

```
========================================
 #  name       shape      position (x, y, z)     size (w x d x h) m
---  --------  ---------  ---------------------  --------------------
 1:  cube      [cube]    (0.35,  0.05, 0.20)    0.04 x 0.04 x 0.04
 2:  cylinder  [cylinder] (0.35, -0.10, 0.20)   0.04 x 0.04 x 0.06
 3:  box       [box]     (0.25,  0.10, 0.20)    0.05 x 0.05 x 0.05
 q:  quit & shutdown
> 
```

### 与 RoboGrasp-Pipeline 联合运行

```bash
# 终端 1: 启动 piper_control (MoveIt Bridge + FSM + 仿真)
cd ../piper_conrtrol
source scripts/setup_env.sh
ros2 launch piper_highlevel piper_moveit_bridge.launch.py

# 终端 2: 启动 Vision Pipeline（键盘交互）
cd ../RoboGrasp-Vision
source scripts/setup_env.sh
./scripts/mvp_interactive.sh

# 在终端 2 按 1/2/3 触发模拟检测，
# 终端 1 的 piper_control FSM 会自动执行完整的 pick-and-place
```

> **注意**：`ros2 launch` 方式下键盘输入无法传递给 `mock_perception` 节点，
> 请使用 `./scripts/mvp_interactive.sh` 或 `ros2 run robograsp_vision_perception mock_perception`。

### 调试

```bash
# 监听各个 topic
ros2 topic echo /perception/result
ros2 topic echo /target_pose

# 用参数直接触发检测（不依赖键盘）
ros2 param set /mock_perception trigger_index 2

# 手动发布 mock 检测（直接运行节点）
ros2 run robograsp_vision_perception mock_perception

# 查看节点图
ros2 run rqt_graph rqt_graph
```

## 与 RoboGrasp-Pipeline 的关系

```
RoboGrasp-Vision (本仓库)         RoboGrasp-Pipeline (piper_control)
┌─────────────────────────┐      ┌──────────────────────────────┐
│  Perception Layer       │      │                              │
│  ┌───────────────────┐  │      │  ┌────────────────────────┐  │
│  │ Object Detection  │  │      │  │  MoveIt2 FSM           │  │
│  │ 3D Localization   │  │      │  │  (Pick & Place)        │  │
│  └────────┬──────────┘  │      │  └──────────▲─────────────┘  │
│           │             │      │             │                │
│  ┌────────▼──────────┐  │      │  ┌──────────┴─────────────┐  │
│  │ Robot Bridge      ├──┼──────┼──► /target_pose              │
│  └───────────────────┘  │      │             │                │
│                         │      │  ┌──────────┴─────────────┐  │
│  Research / Perception  │      │  │  Motion Planning       │  │
│  (Python, fast iterate) │      │  │  (MoveIt + OMPL)       │  │
│                         │      │  └──────────▲─────────────┘  │
│                         │      │             │                │
│                         │      │  ┌──────────┴─────────────┐  │
│                         │      │  │  Hardware Driver        │  │
│                         │      │  │  (piper_ros, CAN)      │  │
└─────────────────────────┘      │  └────────────────────────┘  │
                                 │                              │
                                 │  Manipulation / Execution    │
                                 │  (C++, MoveIt2)              │
                                 └──────────────────────────────┘
```

- **RoboGrasp-Vision = 感知层**（Python，快速研究迭代）
- **RoboGrasp-Pipeline = 执行层**（C++/MoveIt2，稳定操作基座）
- 通过 `/target_pose` 话题解耦，可独立开发、测试、替换

## 扩展计划

| 阶段 | 内容 | 预计依赖 |
|------|------|---------|
| **v0.2** | 接入真实相机 (RealSense D435) | `realsense2_camera` |
| **v0.3** | 接入真实 YOLO 检测 | `ultralytics`, `cv_bridge` |
| **v0.4** | 点云处理 & 6-DoF 抓取生成 | `open3d`, `PCL` |
| **v0.5** | Failure recovery & retry | 内置 |
| **v0.6** | Runtime monitoring dashboard | `rqt`, `plotjuggler` |
| **v1.0** | 多物体场景、主动视角规划 | 待定 |

## 开发指南

### 添加新的感知后端

1. 实现 `ObjectDetector` 子类（参见 `detector.hpp`），在 `detect()` / `detect_by_index()` 中填充 `DetectionResult`（包括 `bbox_size`）
2. 修改 `MockDetector::objects_` 中的数据即可修改模拟物体的位置、尺寸、形状
3. 真实相机后端直接替换检测器即可，下游无需改动

### 添加新的机器人后端

1. 实现 `RobotInterface` 子类（参见 `robot_interface.hpp`）
2. 在 `bridge_node.cpp` 中注册新的 backend 名称

## 许可证

MIT

# RoboGrasp-Vision

**视觉驱动自主抓取研究平台**

RoboGrasp-Vision 是一个**以 C++ 为主**的模块化机器人视觉抓取系统，在 [RoboGrasp-Pipeline (piper_control)](../piper_conrtrol) 的底层操作能力之上，增加感知层与自主决策层，让机器人能够"看见物体 → 估计位置 → 生成抓取指令 → 调用底层执行"。

主要开发语言：**C++ (rclcpp + ament_cmake)**，launch 文件使用 Python。

```
+------------------+     +------------------+     +------------------+
|   Perception     | --> |  Grasp           | --> |   Robot Bridge   |
| (object detect,  |     | Generation       |     | (command relay)  |
|  3D localization)|     | (pose & quality) |     |                  |
+------------------+     +------------------+     +--------+---------+
        |                                                    |
        v                                                    v
+------------------------+                     +------------------------+
|   Sensor Input (mock   |                     | RoboGrasp-Pipeline     |
|   or real RGB-D)       |                     | (piper_control FSM)    |
+------------------------+                     +------------------------+
```

## 目录结构

```
RoboGrasp-Vision/
├── README.md                          # 本文件
├── .gitignore
├── scripts/
│   └── setup_env.sh                   # 环境加载脚本
└── src/
    ├── robograsp_interfaces/          # 自定义 ROS2 消息/服务接口
    │   ├── msg/
    │   │   ├── PerceptionResult.msg   # 感知结果
    │   │   └── GraspCommand.msg       # 抓取指令
    │   └── srv/
    │       └── GraspRequest.srv       # 抓取请求服务
    │
    ├── robograsp_vision_perception/   # 感知层
    │   ├── perception_node.py         # 主感知节点
    │   ├── mock_perception.py         # Mock 感知（开发用）
    │   ├── detector.py                # 检测器抽象 & MockDetector
    │   └── config/
    │       └── perception_params.yaml
    │
    ├── robograsp_vision_grasp/        # 抓取生成层
    │   ├── grasp_node.py              # 抓取节点
    │   ├── generator.py               # 抓取生成器 & SimpleTopDown
    │   └── config/
    │       └── grasp_params.yaml
    │
    ├── robograsp_vision_bridge/       # 机器人桥接层
    │   ├── bridge_node.py             # 桥接节点
    │   ├── robot_interface.py         # 机器人接口 & PiperControlInterface
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
[mock_perception]                     [grasp_node]                     [bridge_node]
  Timer → detect()                      sub: /perception/result          sub: /grasp/command
       ↓ pub: /perception/result              ↓ gen: top-down grasp             ↓ send: PoseStamped
       PerceptionResult                        ↓ pub: /grasp/command             ↓ pub: /target_pose
                                               GraspCommand                           [piper_control FSM]
```

### 模块职责

| 模块 | 职责 | 输入 | 输出 |
|------|------|------|------|
| **perception** | 物体检测、3D 定位 | 相机图像/点云 (或 mock) | `PerceptionResult` |
| **grasp_generation** | 抓取位姿生成、质量评估 | `PerceptionResult` | `GraspCommand` |
| **bridge** | 指令转发、后端适配 | `GraspCommand` | `/target_pose` (PoseStamped) |
| **bringup** | 统一启动入口、参数管理 | — | launch 文件 |

### 关键设计决策

- **接口优先**：各层通过自定义 ROS2 消息松耦合，可独立替换
- **抽象基类**：detector、generator、robot_interface 均为 ABC，支持多后端
- **Mock 先行**：MVP 阶段全 mock 数据，零硬件依赖即可运行完整数据流
- **C++ 为主**：感知、抓取、桥接层均使用 C++ (rclcpp)，与 piper_control 语言栈一致

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
# 终端 1: 启动完整 Vision Pipeline (mock 模式)
ros2 launch robograsp_vision_bringup mvp_demo.launch.py

# 观察输出:
# [mock_perception] Published detection: cube at (0.350, 0.050, 0.020)
# [grasp_node] Generated top grasp for 'cube' at (0.350, 0.050, 0.020)
# [bridge_node] Sent grasp to /target_pose: (0.350, 0.050, 0.020)
```

### 与 RoboGrasp-Pipeline 联合运行

```bash
# 终端 1: 启动 piper_control (MoveIt Bridge + FSM)
cd ../piper_conrtrol
source scripts/setup_env.sh
ros2 launch piper_highlevel piper_moveit_bridge.launch.py

# 终端 2: 启动 Vision Pipeline
cd ../RoboGrasp-Vision
source scripts/setup_env.sh
ros2 launch robograsp_vision_bringup demo.launch.py

# Vision Pipeline 会自动 publish 到 /target_pose，
# piper_control FSM 会自动执行完整的 pick-and-place
```

### 调试

```bash
# 监听各个 topic
ros2 topic echo /perception/result
ros2 topic echo /grasp/command
ros2 topic echo /target_pose

# 手动发布 mock 检测
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
│  │ Grasp Generation  │  │      │  │  Motion Planning       │  │
│  └────────┬──────────┘  │      │  │  (MoveIt + OMPL)       │  │
│           │             │      │  └──────────▲─────────────┘  │
│  ┌────────▼──────────┐  │      │             │                │
│  │ Robot Bridge      ├──┼──────┼──► /target_pose              │
│  └───────────────────┘  │      │             │                │
│                         │      │  ┌──────────┴─────────────┐  │
│  Research / Perception  │      │  │  Hardware Driver        │  │
│  (Python, fast iterate) │      │  │  (piper_ros, CAN)      │  │
└─────────────────────────┘      │  └────────────────────────┘  │
                                 │                              │
                                 │  Manipulation / Execution    │
                                 │  (C++, MoveIt2)              │
                                 └──────────────────────────────┘
```

- **RoboGrasp-Vision = 感知层 + 决策层**（Python，快速研究迭代）
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

1. 实现 `ObjectDetector` 子类（参见 `detector.py`）
2. 修改 `perception_node.py` 中的 `_detector` 实例化
3. 更新 `perception_params.yaml` 的 `use_mock: false`

### 添加新的抓取算法

1. 实现 `GraspGenerator` 子类（参见 `generator.py`）
2. 修改 `grasp_node.py` 中的 `_generator` 实例化

### 添加新的机器人后端

1. 实现 `RobotInterface` 子类（参见 `robot_interface.py`）
2. 在 `bridge_node.py` 中注册新的 backend 名称

## 许可证

MIT

# RoboGrasp-Vision

⚠本仓库仍在开发阶段，目前功能仍没有完全完善，详情可见更新日志模块

**视觉驱动自主抓取研究平台**

RoboGrasp-Vision 是一个**以 C++ 为主**的模块化机器人视觉抓取系统，在 [RoboGrasp-Pipeline (piper_control)](../piper_conrtrol) 的底层操作能力之上，增加感知层与自主决策层，让机器人能够"看见物体 → 估计位置 → 生成抓取指令 → 调用底层执行"。

主要开发语言：**C++ (rclcpp + ament_cmake)**，launch 文件使用 Python。

## 系统架构

### 模块职责

| 模块 | 职责 | 输入 | 输出 |
|------|------|------|------|
| **perception** | 物体检测、3D 定位、坐标系变换 | 相机/参数触发 (sensor frame) | `PerceptionResult` (world frame) |
| **bridge** | 指令转发、机器人后端适配 | `PerceptionResult` (world frame) | `/target_pose` (ObjectInfo, world frame) |
| **bringup** | 统一启动入口、参数管理 | — | launch 文件 |

### 关键设计决策

- **接口优先**：各层通过自定义 ROS2 消息松耦合，可独立替换
- **抽象基类**：detector、robot_interface 均为 ABC，支持多后端
- **C++ 为主**：感知、桥接层均使用 C++ (rclcpp)，与 piper_control 语言栈一致
- **感知层做 TF**：反投影结果拿到后立即 TF 变换到 world 系，后续 service → topic → 终端 → bridge 全链路统一语义，无坐标系断层

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

### 运行 Camera Detection

```bash
# 摄像头检测交互启动
# 使用真实相机（或 Realsense 仿真）进行物体检测
# 启动后通过键盘交互触发检测和抓取
./scripts/camera_interactive.sh
```

此脚本会依次：
1. 加载环境变量
2. 启动 `camera_detect.launch.py`（摄像头检测管线）
3. 运行 `camera_interactive.py`（键盘交互控制）
4. 退出时自动关闭所有节点

> **注意**：使用 `./scripts/camera_interactive.sh`（而非 `ros2 launch`）启动，以保证键盘输入能正确传递给交互控制脚本。

### 与 RoboGrasp-Pipeline 联合运行

```bash
# 终端 1: 启动 piper_control (MoveIt Bridge + FSM + 仿真)
cd ../piper_conrtrol
source scripts/setup_env.sh
ros2 launch piper_highlevel piper_moveit_bridge.launch.py

# 终端 2: 启动 Vision Pipeline
cd ../RoboGrasp-Vision
source scripts/setup_env.sh
./scripts/camera_interactive.sh
```

### 调试

```bash
# 监听各个 topic
ros2 topic echo /perception/result
ros2 topic echo /target_pose

# 查看节点图
ros2 run rqt_graph rqt_graph
```

## 与 RoboGrasp-Pipeline 的关系

- **RoboGrasp-Vision = 感知层**（C++/rclcpp，快速研究迭代）
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
2. 确保 `PerceptionNode` 已通过 TF 将 `position_3d` 从 `sensor_frame` 变换到 `target_frame`（参见 `transform_position()`），下游所有节点依赖 world 系语义
3. 真实相机后端直接替换检测器即可，下游无需改动

### 添加新的机器人后端

1. 实现 `RobotInterface` 子类（参见 `robot_interface.hpp`）
2. 在 `bridge_node.cpp` 中注册新的 backend 名称

## 更新日志

### 2026-05-30

- 新增深度几何检测器，基于高度图分割与形状分析，无需颜色先验
- 新增检测后端可插拔切换（`color_threshold` / `depth_geometry`）
- 新增 HSV 颜色阈值参数化配置
- 修复 `perception_node` 共享库链接失败
- 优化 3D 定位精度，改用轮廓内深度中位数替代单像素反投影
- 修复物体底面位置计算偏差
- 精简 bridge 包依赖

- 由于HSV-D的检测方法对于圆柱检测困难，目前只有box物体位置相对稳定，后续会针对补足

MIT

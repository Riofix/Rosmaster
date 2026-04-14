# Rosmaster
2026年中国大学生机械工程创新创意大赛

# src 目录结构说明

src/
    ├── robot_bringup/       # 启动中心：存放所有 launch 文件和配置文件
    ├── robot_description/   # 模型中心：存放 URDF 模型、坐标系 (TF) 定义
    ├── robot_interfaces/    # 自定义接口：存放特殊的串口通信协议或传感器消息
    ├── robot_base/          # 底盘驱动：即你现在的串口通信节点
    ├── robot_sensors/       # 传感器驱动：后续的雷达、摄像头驱动配置
    └── robot_navigation/    # 导航配置：存放 Nav2 的参数文件和地图

---

# robot_base 模块重构与底层通信协议升级 (v1.0.0)

## 📌 更新摘要

本次更新对 robot_base（底盘控制包）进行了全面重构，核心目标是实现 “控制逻辑与协议解析绝对解耦”。

通过引入三层架构设计，模块现已完美支持基于官方 CSV 通信协议的 100% 指令解析，同时为未来的 Nav2 导航栈接入提供了标准的里程计（Odometry）和 TF 坐标变换支持。

---

## 🏗️ 架构设计：三层解耦模型

为了保持框架的整洁与高内聚低耦合，底盘驱动被严格划分为以下三个独立层级：

### 1. 协议层 (protocol_parser.py)
- 性质：纯逻辑层
- 功能：只负责字节流的封包（Pack）和解包（Unpack），处理十六进制、小端序、校验和计算以及小数的倍数缩放
- 特点：
  - 零外部依赖（无 ROS、无 Serial 库）
  - 纯 Python 实现，极易进行独立的单元测试
  - 覆盖了 CSV 文件中所有的底盘、机械臂、PWM、PID 等指令

### 2. 驱动层 (serial_driver.py)
- 性质：硬件交互层
- 功能：向下管理串口生命周期，向上提供面向对象的 Python API（如 set_velocity）
- 特点：
  - 内置独立后台读取线程与线程锁（threading.Lock）
  - 从底层彻底解决了串口通信中常见的“粘包”与“半包”问题
  - 通过事件回调机制（Callback）将解析好的干净数据（Dict）推流给上层

### 3. ROS 层 (base_node.py)
- 性质：业务逻辑层
- 功能：继承自 rclpy.node.Node，作为 ROS 2 世界与物理底盘的桥梁
- 特点：
  - 订阅 /cmd_vel，实现键盘/导航栈对小车速度的控制
  - 接收底盘回传的实时速度，利用积分推算机器人的世界坐标 (x, y, θ)
  - 发布标准的 nav_msgs/Odometry 消息与 odom → base_link 的 TF 树
  - 满足 Nav2 自主行走的核心前置要求

---

## 🔌 硬件绑定与串口配置（重要！）

为了解决 Linux 系统下 USB 设备多次插拔导致的串口号漂移（如 ttyUSB0 变为 ttyUSB1）问题，本项目已在底层将设备串口固定绑定为：

👉 /dev/rosmaster

### 开发与部署须知：
- 节点启动时，默认将自动连接 /dev/rosmaster
- 在新设备上部署时，请务必提前配置 udev 规则，将主控板的 USB 端口映射为 /dev/rosmaster，并赋予 0666 权限

---

## 🚀 快速测试指南

### 1. 编译工作空间

cd ~/robot_ws
colcon build --packages-select robot_base
source install/setup.bash

### 2. 启动底盘核心节点

ros2 run robot_base base_node
# 预期输出: [Driver] Successfully connected to /dev/rosmaster at 115200 baud.

### 3. 测试键盘遥控

打开新终端运行：

ros2 run teleop_twist_keyboard teleop_twist_keyboard

按下 i, j, l, , 即可控制底盘移动。

### 4. 验证导航前置数据（里程计与 TF）

打开新终端，查看坐标推算是否正常工作：

ros2 topic echo /odom

---

## 💡 维护建议

未来若更换了底盘主板或协议升级，仅需修改 protocol_parser.py 文件即可，无需改动任何 ROS 节点与导航配置代码！
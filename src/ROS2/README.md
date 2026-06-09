# 🤖 ROS 2 上位机软件栈

**2026年中国大学生机械工程创新创意大赛 —— 智能机器人 ROS 2 控制系统**

本目录包含运行于 **RDK X5 / 工控机** 上的 ROS 2 (Humble) 软件包，构成完整的机器人上位机控制系统。

---

## 📦 包结构总览

```
src/ROS2/
├── README.md                       # 本文件
├── robot_brain/                    # 🧠 大脑决策层
├── robot_control/                  # 🎮 控制中台
├── robot_fusion/                   # 🔗 融合层
├── robot_link/                     # 📡 链路层
├── robot_protocol/                 # 📦 协议层
├── robot_vision/                   # 👁️ 视觉层
└── robot_bringup/                  # 🚀 启动配置
```

### 数据流架构

```
                   ┌──────────────────────┐
                   │   BrainNode          │  ← 顶层状态机 (7步自动化)
                   │  (robot_brain)       │
                   └──────────┬───────────┘
                              │ /brain_cmd (JSON)
                              ▼
                   ┌──────────────────────┐
                   │  ControlNode         │  ← 语义指令 → 硬件控制
                   │  (robot_control)     │     PID 闭环调节
                   └────┬─────────────┬───┘
                        │             │
              /control_cmd (JSON)    /protocol_internal_cmd (JSON)
                        │             │
                        ▼             ▼
          ┌─────────────────────┐   ┌──────────────────┐
          │ ProtocolPackNode    │   │ ProtocolNode      │
          │ (封包: JSON→二进制) │   │ (解析: 二进制→JSON)│
          │ (robot_protocol)    │   │ (robot_protocol)   │
          └────────┬────────────┘   └─────────┬─────────┘
                   │                          │
          /serial_tx_raw /tcp_tx_raw  /serial_rx_raw /tcp_rx_raw
          (UInt8MultiArray)           (UInt8MultiArray)
                   │                          │
                   ▼                          ▼
          ┌──────────────────┐   ┌──────────────────────────┐
          │ SerialNode       │   │ TcpServerNode            │
          │ (USB串口: /dev/  │   │ (TCP Server: 端口 8080)  │
          │  rosmaster)      │   │ (robot_link)             │
          │ (robot_link)     │   └────────────┬─────────────┘
          └────────┬─────────┘                │
                   │                          │
                   ▼                          ▼
            ┌──────────────┐     ┌──────────────────────┐
            │ STM32 底盘   │     │ Handle_L/M/R (TCP)   │
            │ (串口通信)    │     │ 步进电机+舵机+无刷   │
            └──────────────┘     └──────────────────────┘

                         ← 融合层聚合所有状态 →
                   ┌──────────────────────────┐
                   │  FusionNode              │
                   │  全局影子状态 50Hz 更新   │
                   │  (robot_fusion)          │
                   └────────────┬─────────────┘
                                │ /world_state (JSON)
                                ▼
                   ┌──────────────────────────┐
                   │  VisionNode              │
                   │  双目摄像机数字识别       │
                   │  (robot_vision)          │
                   └────────────┬─────────────┘
                                │ /vision_detections (JSON)
                                ▼
                          FusionNode 消费
```

---

## 1. 📡 链路层 — `robot_link`

与硬件物理层通信的桥梁，负责原始字节流的收发。

### 节点列表

| 节点 | 文件 | 功能 |
|:---|:---|:---|
| **SerialNode** | `serial_node.py` | 通过 USB 串口 (115200, 8N1) 连接 STM32 底盘，收发 `/dev/rosmaster` |
| **TcpServerNode** | `tcp_server_node.py` | TCP 服务器 (端口 8080)，接收三个抓手 (ESP8266 WiFi) 的连接 |
| **HotspotNode** | `hotspot_node.py` | 自动配置 WiFi 热点 (SSID: Digua)，使用 nmcli 管理网络 |

### SerialNode 参数

| 参数 | 默认值 | 说明 |
|:---|:---|:---|
| `port` | `/dev/rosmaster` | 串口设备路径 (USB 固定映射) |
| `baudrate` | 115200 | 波特率 |

> **⚠️ 重要**: USB 串口已固定映射为 `/dev/rosmaster`，部署新设备前须配置 `udev` 规则。

### TcpServerNode 参数

| 参数 | 默认值 | 说明 |
|:---|:---|:---|
| `port` | 8080 | TCP 监听端口 |
| `ip_left` | `192.168.1.101` | 左抓手 IP |
| `ip_mid` | `192.168.1.102` | 中抓手 IP |
| `ip_right` | `192.168.1.103` | 右抓手 IP |

### HotspotNode 参数

| 参数 | 默认值 | 说明 |
|:---|:---|:---|
| `ssid` | Digua | WiFi 热点名称 |
| `password` | 12345678 | 密码 |
| `interface` | wlan0 | 无线网卡接口 |
| `ip_address` | 192.168.1.1/24 | 静态 IP 段 |

### 话题接口

| 话题 | 类型 | 方向 | 说明 |
|:---|:---|:---:|:---|
| `/serial_rx_raw` | UInt8MultiArray | 发布 | 串口接收的原始字节 |
| `/serial_tx_raw` | UInt8MultiArray | 订阅 | 待发送到串口的原始字节 |
| `/tcp_rx_raw` | UInt8MultiArray | 发布 | TCP 接收的原始字节 (首字节为设备ID) |
| `/tcp_tx_raw` | UInt8MultiArray | 订阅 | 待发送到 TCP 的原始字节 (首字节为目标设备ID) |

---

## 2. 📦 协议层 — `robot_protocol`

负责二进制流与结构化数据之间的双向转换，实现通信协议的完全解耦。

### ProtocolNode (解析节点)

将来自硬件的 `0xFF 0xFB` 帧解析为 JSON 格式的影子状态。

**帧格式 (上行: 硬件 → 上位机):**
```
0xFF 0xFB | LEN | CMD | DATA... | CS
```

**解析表驱动架构:**
- **Handle (抓手) 指令解析**: `0x5A`(MPU姿态) / `0x5B`(步进电机里程计) / `0x5C`(PWM状态) / `0x5D`(颜色ID)
- **Chassis (底盘) 指令解析**: `0x0A`(底盘状态) / `0x0E`(IMU原始数据) / `0x0C`(IMU欧拉角) / `0x0D`(电机编码器)

**影子状态结构:**
- `handle_states`: 每个抓手的 MPU 姿态、步进电机状态、PWM 占空比、舵机角度、颜色 ID、到位标志等
- `chassis_state`: 底盘速度、电压、IMU 六轴数据、欧拉角、电机编码器、到位标志等

### ProtocolPackNode (封包节点)

将上层控制指令 (JSON) 打包为 `0xFF 0xFC` 协议帧并路由到对应硬件。

**帧格式 (下行: 上位机 → 硬件):**
```
0xFF 0xFC | LEN | CMD | sub_id | PARAMS... | CS
```

**路由规则:** 目标设备 ID 0-2 → TCP (抓手), ID 3 → 串口 (底盘)

### 话题接口

| 话题 | 类型 | 发布/订阅 | 说明 |
|:---|:---|:---:|:---|
| `/robot_shadow_states` | String | 发布 | 解析后的设备状态 (JSON) |
| `/control_cmd` | String | 订阅 | 控制指令 (JSON) → 封包发送 |
| `/protocol_internal_cmd` | String | 订阅 | 内部状态强制更新指令 |
| `/tcp_rx_raw` | UInt8MultiArray | 订阅 | TCP 原始字节输入 |
| `/serial_rx_raw` | UInt8MultiArray | 订阅 | 串口原始字节输入 |
| `/tcp_tx_raw` | UInt8MultiArray | 发布 | TCP 原始字节输出 |
| `/serial_tx_raw` | UInt8MultiArray | 发布 | 串口原始字节输出 |

---

## 3. 🔗 融合层 — `robot_fusion`

**FusionNode** — 数据融合中心，聚合所有物理设备状态和视觉识别结果。

- 订阅 `/robot_shadow_states` (来自 ProtocolNode) 和 `/vision_detections` (来自 VisionNode)
- 维护统一的 **全局影子状态 (World State)**，以 50Hz 频率发布到 `/world_state`
- 为大脑决策层 (BrainNode) 提供上帝视角的完整状态视图

### 全局状态结构

```json
{
  "handles": {
    "handle_left": { "mpu": {}, "stepmotor": {}, "color_id": 0, ... },
    "handle_mid": { ... },
    "handle_right": { ... }
  },
  "chassis": { "speed": {}, "imu_euler": {}, "motor_encoder": [], ... },
  "vision": { "sequence": [1,2,3,4,5], "status": "confirmed" },
  "last_heartbeat": 1234567890.0
}
```

---

## 4. 🎮 控制层 — `robot_control`

**ControlNode** — 指令中台，桥接大脑的语义指令与具体的硬件控制信号。

### 核心能力

- **语义指令映射**: 将大脑层发出的 "move_to"、"start_cycle" 等高级指令转换为具体硬件命令
- **工业级 PID 控制器**: 内置 PositionPID 类，支持 Kp/Ki/Kd 参数配置、积分限幅和加速度斜坡限制
- **TaskID 生命周期管理**: 通过 `/protocol_internal_cmd` 同步控制状态与影子状态
- **底盘闭环控制**: 订阅影子状态中的编码器反馈，实时 PID 计算速度指令

### PID 参数

| 参数 | 默认值 | 说明 |
|:---|:---:|:---|
| Kp | 0.28 | 比例系数 |
| Ki | 0.08 | 积分系数 |
| Kd | 0.1 | 微分系数 |
| Max Output | 600 | 最大输出 |
| Max Integral | 200 | 积分限幅 |
| Max Accel | 1200 | 最大加速度 (斜坡限制) |

### 硬件设备树

```python
device_tree = {
    "chassis":   { "base":  {"sub_id": 0x01, "type": "mecanum_base"} },
    "handle_left": {
        "track": {"sub_id": 0x01, "type": "handle_servo"},  # 水平轨道
        "lift":  {"sub_id": 0x02, "type": "handle_servo"},  # 垂直升降
        "grab":  {"sub_id": 0x03, "type": "handle_bldc"}    # 无刷抓取
    }
    # handle_mid, handle_right 同理...
}
```

### 辅助工具 — `demo_rosmaster.py`

提供基于 PySide6 的桌面调试界面，包含:
- 串口选择与连接管理
- PID 参数实时调节与可视化
- 底盘速度键盘/滑条控制
- 舵机、无刷电机手动测试

---

## 5. 🧠 大脑层 — `robot_brain`

**BrainNode** — 顶层决策状态机，实现完整的竞赛自动化流程。

### 状态机流程

```
ST_INIT ──→ ST_WAIT_VIS ──→ ST_MOVE_WORK ──→ ST_GRABBING
  │                                                   │
  │              ST_FINISHED ←── ST_UNLOADING ←─── ST_MOVE_DROP
  │                  │
  └── 所有设备就绪 ──┘
```

| 状态 | 说明 |
|:---|:---|
| **ST_INIT** (0) | 设备在线自检，等待所有硬件节点就绪 |
| **ST_WAIT_VIS** (1) | 等待视觉节点识别结果 (数字序列 1-5 的排列) |
| **ST_MOVE_WORK** (2) | 底盘前进至 1.5m 作业区，抓手横向对准轨道 |
| **ST_GRABBING** (3) | 执行垂直抓取循环: 下降 → 无刷电机吸抓 → 上升 |
| **ST_MOVE_DROP** (4) | 动态映射计算: 根据颜色→序列槽位映射，前往放置区 |
| **ST_UNLOADING** (5) | 舵机 180° 翻转卸料 |
| **ST_FINISHED** (6) | 任务完成，返回原点 |

### 指令格式

```json
{
  "device": "chassis",
  "subsystem": "stepper_main",
  "action": "move_to",
  "params": { "pos": 1500 }
}
```

---

## 6. 👁️ 视觉层 — `robot_vision`

**VisionNode** — 双目摄像头远距离数字序列识别节点。

### 识别流程

1. 双摄像头同时采集帧画面 (右摄像头旋转 180° 对齐)
2. 透视变换 (Perspective Transform) 矫正区域
3. 帧序列堆叠降噪 (加权平均)
4. 自适应二值化 + 中值滤波
5. 轮廓检测 + 中心距离筛选
6. 模板匹配 (TM_CCOEFF_NORMED) 数字分类
7. 左右结果融合 → 补全缺失数字 → 历史投票 → 去重

### 识别结果

```json
{
  "sequence": ["3", "1", "5", "2", "4"],
  "mode": "success"
}
```

### 配置参数

配置文件位于 `robot_vision/config/visison_params.yaml`:
- 透视变换目标尺寸、ROI 区域定义
- 预处理参数 (中值滤波核、二值化阈值)
- 帧堆叠数量、历史投票窗口大小
- 超时重试策略

---

## 7. 🚀 启动层 — `robot_bringup`

**hardware_launch.py** — 一键启动所有硬件相关节点。

```bash
ros2 launch robot_bringup hardware_launch.py
```

启动顺序:
1. `tcp_server_node` — TCP 服务器 (抓手通信)
2. `serial_node` — 串口通信 (底盘通信)
3. `protocol_node` — 协议解析
4. `protocol_pack_node` — 协议封包
5. `fusion_node` — 数据融合
6. `control_node` — 控制中台
7. `brain_node` — 大脑决策
8. `vision_node` — 视觉识别

---

## 🚀 快速开始

```bash
# 1. 编译
cd ~/robot_ws
colcon build --symlink-install
source install/setup.bash

# 2. 一键启动所有硬件节点
ros2 launch robot_bringup hardware_launch.py

# 3. 查看融合层状态
ros2 topic echo /world_state

# 4. 单独测试视觉（可选）
ros2 run robot_vision vision_node

# 5. 查看视觉识别结果
ros2 topic echo /vision_detections
```

---

## 📋 话题总览

| 话题 | 类型 | 频率 | 说明 |
|:---|:---|:---:|:---|
| `/world_state` | String | 50Hz | 全局影子状态 (FusionNode) |
| `/brain_cmd` | String | 10Hz | 大脑决策指令 (BrainNode) |
| `/control_cmd` | String | - | 控制指令 (ControlNode→PackNode) |
| `/protocol_internal_cmd` | String | - | 内部状态强制更新 |
| `/robot_shadow_states` | String | 高 | 协议解析后设备状态 (ProtocolNode) |
| `/vision_detections` | String | - | 视觉识别结果 (VisionNode) |
| `/serial_rx_raw` | UInt8MultiArray | 高 | 串口原始字节入 |
| `/serial_tx_raw` | UInt8MultiArray | - | 串口原始字节出 |
| `/tcp_rx_raw` | UInt8MultiArray | 高 | TCP 原始字节入 |
| `/tcp_tx_raw` | UInt8MultiArray | - | TCP 原始字节出 |

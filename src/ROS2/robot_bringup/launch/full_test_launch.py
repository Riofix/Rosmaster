"""
全链路测试 Launch — 串口(底盘) + TCP(抓手) + 协议 + 融合 + 控制
不启动 brain/vision/hotspot — 避免状态机自动运行

启动节点:
  1. serial_node       → 串口通信 (底盘 /dev/Rosmaster)
  2. tcp_server_node   → TCP 服务器 (抓手, 端口 3456)
  3. protocol_node     → 上行解析 (FF FB → JSON)
  4. protocol_pack_node→ 下行封包 (JSON → FF FC)
  5. fusion_node       → 状态融合 → /world_state (50Hz)
  6. control_node      → PID 控制 + 指令映射

用法:
  ros2 launch robot_bringup full_test_launch.py

连接后查看日志确认抓手 IP, 修改本文件中 ip_left/mid/right 后重新启动

底盘测试 (务必加 --once):
  # 放置区
  ros2 topic pub --once /brain_cmd std_msgs/msg/String \
    '{"data":"{\"device\":\"chassis\",\"subsystem\":\"base\",\"action\":\"move_to\",\"task_id\":1,\"params\":{\"pos\":30375}}"}'
  # 抓取区
  ros2 topic pub --once /brain_cmd std_msgs/msg/String \
    '{"data":"{\"device\":\"chassis\",\"subsystem\":\"base\",\"action\":\"move_to\",\"task_id\":1,\"params\":{\"pos\":-47628}}"}'
  # 急停
  ros2 topic pub --once /brain_cmd std_msgs/msg/String \
    '{"data":"{\"device\":\"chassis\",\"subsystem\":\"base\",\"action\":\"stop\",\"task_id\":1,\"params\":{}}"}'
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ── 链路层 ──────────────────────────────────────────
        # 1. 串口通信 (底盘 USB)
        Node(
            package='robot_link',
            executable='serial_node',
            name='serial_node',
            parameters=[{
                'port': '/dev/Rosmaster',
                'baudrate': 115200,
            }],
            output='screen',
        ),

        # 2. TCP 服务器 (三抓手 WiFi)
        #    连接后看日志确认 IP, 修改 ip_left/mid/right
        Node(
            package='robot_link',
            executable='tcp_server_node',
            name='tcp_server_node',
            parameters=[{
                'port': 3456,
                'ip_left': '10.245.159.101',     # TODO: 确认后修改
                'ip_mid': '10.245.159.102',      # TODO: 确认后修改
                'ip_right': '10.245.159.103',    # TODO: 确认后修改
            }],
            output='screen',
        ),

        # ── 协议层 ──────────────────────────────────────────
        # 3. 上行解析 (FF FB → /robot_shadow_states)
        Node(
            package='robot_protocol',
            executable='protocol_node',
            name='protocol_node',
            output='screen',
        ),

        # 4. 下行封包 (/control_cmd → FF FC)
        Node(
            package='robot_protocol',
            executable='protocol_pack_node',
            name='protocol_pack_node',
            output='screen',
        ),

        # ── 融合层 ──────────────────────────────────────────
        # 5. 状态融合 (/robot_shadow_states → /world_state)
        Node(
            package='robot_fusion',
            executable='fusion_node',
            name='fusion_node',
            output='screen',
        ),

        # ── 控制层 ──────────────────────────────────────────
        # 6. 控制中台 (/brain_cmd → PID 闭环 → /control_cmd)
        Node(
            package='robot_control',
            executable='control_node',
            name='control_node',
            output='screen',
        ),

        # ⚠️ 不启动 brain_node (状态机)
        # ⚠️ 不启动 vision_node (视觉)
        # ⚠️ 不启动 hotspot_node (热点)
    ])

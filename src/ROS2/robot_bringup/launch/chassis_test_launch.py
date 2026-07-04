"""
底盘测试 Launch — 仅启动串口链路 + 控制层，不启动状态机/视觉/TCP。

启动节点:
  1. serial_node       → 串口通信 (底盘 USB)
  2. protocol_node     → 上行解析 (FF FB → JSON)
  3. protocol_pack_node→ 下行封包 (JSON → FF FC)
  4. fusion_node       → 状态融合 → /world_state (50Hz)
  5. control_node      → PID 控制 + 指令映射

用法:
  ros2 launch robot_bringup chassis_test_launch.py

测试指令 (务必加 --once, 否则持续循环发送):
  # 放置区 +30375 (W方向)
  ros2 topic pub --once /brain_cmd std_msgs/msg/String \
    '{"data":"{\"device\":\"chassis\",\"subsystem\":\"base\",\"action\":\"move_to\",\"task_id\":1,\"params\":{\"pos\":30375}}"}'

  # 抓取区 -47628 (S方向)
  ros2 topic pub --once /brain_cmd std_msgs/msg/String \
    '{"data":"{\"device\":\"chassis\",\"subsystem\":\"base\",\"action\":\"move_to\",\"task_id\":1,\"params\":{\"pos\":-47628}}"}'

  # 急停
  ros2 topic pub --once /brain_cmd std_msgs/msg/String \
    '{"data":"{\"device\":\"chassis\",\"subsystem\":\"base\",\"action\":\"stop\",\"task_id\":1,\"params\":{}}"}'

到位监控:
  ros2 topic echo /world_state | grep -A2 arrival_done
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

        # ── 协议层 ──────────────────────────────────────────
        # 2. 上行解析 (FF FB → /robot_shadow_states)
        Node(
            package='robot_protocol',
            executable='protocol_node',
            name='protocol_node',
            output='screen',
        ),

        # 3. 下行封包 (/control_cmd → FF FC)
        Node(
            package='robot_protocol',
            executable='protocol_pack_node',
            name='protocol_pack_node',
            output='screen',
        ),

        # ── 融合层 ──────────────────────────────────────────
        # 4. 状态融合 (/robot_shadow_states → /world_state)
        Node(
            package='robot_fusion',
            executable='fusion_node',
            name='fusion_node',
            output='screen',
        ),

        # ── 控制层 ──────────────────────────────────────────
        # 5. 控制中台 (/brain_cmd → PID 闭环 → /control_cmd)
        Node(
            package='robot_control',
            executable='control_node',
            name='control_node',
            output='screen',
        ),

        # ⚠️ 不启动 brain_node (状态机)
        # ⚠️ 不启动 vision_node (视觉)
        # ⚠️ 不启动 tcp_server_node / hotspot_node (底盘只走串口)
    ])

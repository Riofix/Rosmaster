"""
全链路测试 Launch — 全部节点, 默认单步模式

启动节点:
  1. serial_node       → 串口通信 (底盘 /dev/Rosmaster)
  2. tcp_server_node   → TCP 服务器 (抓手, 端口 3456)
  3. protocol_node     → 上行解析 (FF FB → JSON)
  4. protocol_pack_node→ 下行封包 (JSON → FF FC)
  5. fusion_node       → 状态融合 → /world_state (50Hz)
  6. control_node      → PID 控制 + 指令映射
  7. brain_node        → 状态机 (默认单步, step_next 推进)
  8. vision_node       → 视觉识别 (debug 窗口关闭)
  9. voice_node        → 语音播报与指令识别 (/dev/broadcast)

测试指令:
  ros2 topic pub --once /task_control ... '{"cmd":"step_next"}'
  ros2 topic pub --once /task_control ... '{"cmd":"reset"}'
  ros2 topic pub --once /task_control ... '{"cmd":"estop"}'

用法:
  ros2 launch robot_bringup full_test_launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    debug_mode = LaunchConfiguration('debug_mode', default='True')
    return LaunchDescription([
        DeclareLaunchArgument('debug_mode', default_value='True'),

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
                'ip_left': '10.245.159.251',
                'ip_mid': '10.245.159.29',
                'ip_right': '10.245.159.17',
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

        # ── 视觉层 ──────────────────────────────────────────
        # 7. 视觉识别
        Node(
            package='robot_vision',
            executable='vision_node',
            name='vision_node',
            parameters=[{'debug_mode': False, 'broadcast_enabled': False}],
            output='screen',
        ),

        # ── 大脑层 ──────────────────────────────────────────
        # 8. 状态机 (debug_mode=true → 单步, 可命令行覆盖)
        Node(
            package='robot_brain',
            executable='brain_node',
            name='brain_node',
            parameters=[{'debug_mode': debug_mode}],
            output='screen',
        ),

        # ── 语音层 ──────────────────────────────────────────
        # 9. 语音播报与指令识别 (SU-03T, /dev/broadcast)
        Node(
            package='robot_voice',
            executable='voice_node',
            name='voice_node',
            parameters=[{
                'port': '/dev/broadcast',
                'baudrate': 115200,
            }],
            output='screen',
        ),

        # ⚠️ 不启动 hotspot_node (热点)
    ])

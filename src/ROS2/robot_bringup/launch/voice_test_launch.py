"""
语音模块专项测试 Launch — 无需下位机, 仅需语音模块

启动节点:
  1. voice_node   → 语音播报 & 指令识别 (/dev/broadcast)
  2. brain_node   → 状态机 (mock_hardware 模式, 跳过硬件初始化)

测试流程:
  1. 启动后自动播报 "好的，已初始化完成"
  2. 对着语音模块说 "地瓜启动" → brain 进入下一状态

用法:
  ros2 launch robot_bringup voice_test_launch.py

手动测试指令:
  # 模拟语音播报
  ros2 topic pub --once /voice_broadcast std_msgs/msg/String '{"data":"{\"cmd_id\":21}"}'

  # 模拟语音启动命令
  ros2 topic pub --once /voice_cmd std_msgs/msg/String '{"data":"{\"cmd_id\":20}"}'

  # 监控语音指令
  ros2 topic echo /voice_cmd
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # 1. 语音层 — SU-03T 语音模块
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

        # 2. 大脑层 — mock_hardware 模式 + 自动运行 (非单步)
        Node(
            package='robot_brain',
            executable='brain_node',
            name='brain_node',
            parameters=[{
                'debug_mode': False,      # 自动运行，不暂停
                'mock_hardware': True,    # 跳过硬件初始化
            }],
            output='screen',
        ),
    ])

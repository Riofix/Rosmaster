from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 链路层 (robot_link) - TCP 服务器
        Node(
            package='robot_link',
            executable='tcp_server_node',
            name='tcp_server_node',
            parameters=[{
                'port': 8080,
                'ip_left': '192.168.1.101',
                'ip_mid': '192.168.1.102',
                'ip_right': '192.168.1.103'
            }],
            output='screen'
        ),

        # 2. 链路层 (robot_link) - 串口底盘
        Node(
            package='robot_link',
            executable='serial_node',
            name='serial_node',
            parameters=[{
                'port': '/dev/rosmaster',
                'baudrate': 115200
            }],
            output='screen'
        ),

        # 3. 解析层 (robot_protocol) - 处理 FF FB 状态解析
        Node(
            package='robot_protocol',
            executable='protocol_node',
            name='protocol_node',
            output='screen'
        ),

        # 4. 封包层 (robot_protocol) - 处理 FF FC 下发打包
        # 注意：确认你的 setup.py 里 executable 名字是 protocol_pack_node
        Node(
            package='robot_protocol',
            executable='protocol_pack_node',
            name='protocol_pack_node',
            output='screen'
        ),

        # 5. 融合层 (robot_fusion) - 维护全局影子状态
        Node(
            package='robot_fusion',
            executable='fusion_node',
            name='fusion_node',
            output='screen'
        ),

        # 6. 控制层 (robot_control) - 语义映射中台
        Node(
            package='robot_control',
            executable='control_node',
            name='control_node',
            output='screen'
        ),

        # 7. 大脑层 (robot_brain) - 自动化 7 步逻辑
        Node(
            package='robot_brain',
            executable='brain_node',
            name='brain_node',
            output='screen'
        ),

        # 8. 视觉层 (robot_vision) - 1.5米远距离识别
        Node(
            package='robot_vision',
            executable='vision_node',
            name='vision_node',
            output='screen'
        ),

        # 9. 语音层 (robot_voice) - SU-03T 语音播报与指令识别
        Node(
            package='robot_voice',
            executable='voice_node',
            name='voice_node',
            parameters=[{
                'port': '/dev/broadcast',
                'baudrate': 115200,
            }],
            output='screen'
        ),
    ])
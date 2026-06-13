from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="=== 启动键盘遥控控制 ==="),

        # 1. 串口链路层 — USB转串口连接STM32底盘
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

        # 2. 协议封包层 — JSON → 0xFF 0xFC 二进制帧
        Node(
            package='robot_protocol',
            executable='protocol_pack_node',
            name='protocol_pack_node',
            output='screen'
        ),

        # 3. Twist 桥接 — /cmd_vel → /control_cmd
        #    car_type: 0x07 = CAR_FOURWHEEL_SAME_DIR (四轮全同向)
        Node(
            package='robot_control',
            executable='twist_to_control_bridge',
            name='twist_to_control_bridge',
            parameters=[{
                'car_type': 4,          # 0x07 四轮全同向
                'linear_scale': 1000,   # m/s → mm/s
                'angular_scale': 2000,  # rad/s → chassis Vz
                'max_vx': 1000,
                'max_vz': 3000,
            }],
            output='screen'
        ),

        # 4. ROS2官方键盘遥控节点 (需先安装: sudo apt install ros-${ROS_DISTRO}-teleop-twist-keyboard)
        #    注意：此节点需要独占终端接收键盘输入，建议在另一个终端手动启动:
        #      ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
        #    如果要在 launch 内启动，取消下面注释:
        # Node(
        #     package='teleop_twist_keyboard',
        #     executable='teleop_twist_keyboard',
        #     name='teleop_twist_keyboard',
        #     prefix='xterm -e',  # 如有 xterm 可用，自动弹窗
        #     parameters=[{
        #         'speed': 0.5,
        #         'turn': 1.0,
        #         'speed_limit': 1.0,
        #         'turn_limit': 1.5,
        #     }],
        #     output='screen',
        # ),

        LogInfo(msg="=== 底层链路已启动，请在新终端执行: ==="),
        LogInfo(msg="  ros2 run teleop_twist_keyboard teleop_twist_keyboard"),
        LogInfo(msg="键位: i=前进 ,=后退 j=左转 l=右转 k=急停 q/z=加减速"),
    ])

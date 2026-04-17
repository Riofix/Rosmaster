import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # ================= 1. 环境准备 =================
    # 网管热点预备，保证所有吊具有网可上
    hotspot_node = Node(
        package='robot_link',
        executable='hotspot_node',
        name='hotspot_node',
        output='screen',
        parameters=[{'ssid': 'Digua', 'password': '12345678'}]
    )
    ld.add_action(hotspot_node)

    # ================= 2. 底盘驱动线 (Rosmaster) =================
    serial_link = Node(
        package='robot_link',
        executable='serial_node',
        name='serial_link',
        output='screen',
        parameters=[{'port': '/dev/rosmaster', 'baudrate': 115200}]
    )
    chassis_parser = Node(
        package='robot_protocol',
        executable='chassis_parser',
        name='chassis_parser',
        output='screen'
    )
    chassis_packer = Node(
        package='robot_protocol',
        executable='chassis_packer',
        name='chassis_packer',
        output='screen'
    )
    ld.add_action(serial_link)
    ld.add_action(chassis_parser)
    ld.add_action(chassis_packer)

    # ================= 3. 吊具集群网络线 =================
    # TCP 服务器承接 3 条 ESP8266 专线
    tcp_server = Node(
        package='robot_link',
        executable='tcp_server_node',
        name='tcp_server_node',
        output='screen',
        parameters=[{'port': 8080}]
    )
    ld.add_action(tcp_server)

    # 利用 ROS2 全分布式多开特性启动 3 对解析层机器人 (参数隔离)
    cranes = ['crane_left', 'crane_middle', 'crane_right']
    for crane_id in cranes:
        c_parser = Node(
            package='robot_protocol',
            executable='crane_parser',
            name=f'{crane_id}_parser',
            output='screen',
            parameters=[{'crane_id': crane_id}]
        )
        c_packer = Node(
            package='robot_protocol',
            executable='crane_packer',
            name=f'{crane_id}_packer',
            output='screen',
            parameters=[{'crane_id': crane_id}]
        )
        c_controller = Node(
            package='robot_control',
            executable='crane_controller',
            name=f'{crane_id}_controller',
            output='screen',
            parameters=[{'crane_id': crane_id}]
        )
        ld.add_action(c_parser)
        ld.add_action(c_packer)
        ld.add_action(c_controller)

    """
    此 Launch 脚本在瞬间并发启动了 11 个独立隔离的进程:
    1 (Hotspot) + 1 (TCP) + 1 (Serial)
    + 2 (Chassis 解析包头) 
    + 6 (3台起重机 各自的拆装包员)
    完美避免一切跨线程错误！
    """
    return ld

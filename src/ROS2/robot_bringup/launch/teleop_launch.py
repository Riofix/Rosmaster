from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='robot_link', executable='tcp_server_node',
             name='tcp_server_node',
             parameters=[{'port': 3456}],
             output='screen'),

        Node(package='robot_link', executable='serial_node',
             name='serial_node',
             parameters=[{'port': '/dev/Rosmaster', 'baudrate': 115200}],
             output='screen'),

        Node(package='robot_protocol', executable='protocol_node',
             name='protocol_node', output='screen'),

        Node(package='robot_protocol', executable='protocol_pack_node',
             name='protocol_pack_node', output='screen'),

        Node(package='robot_fusion', executable='fusion_node',
             name='fusion_node', output='screen'),

        Node(package='robot_control', executable='control_node',
             name='control_node', output='screen'),

        Node(package='robot_control', executable='keyboard_teleop',
             name='keyboard_teleop', output='screen'),
    ])

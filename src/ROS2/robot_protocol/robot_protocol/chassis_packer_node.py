#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8MultiArray, Bool
from .protocol_core import ChassisCodec

class ChassisPackerNode(Node):
    """
    专门针对主逻辑底盘的打包节点
    接受大脑 /cmd_vel 的标准 ROS 数据流，转化为 0xFF 0xFC 协议推至下位机串口
    """
    def __init__(self):
        super().__init__('chassis_packer_node')
        self.tx_pub = self.create_publisher(UInt8MultiArray, '/serial/tx_raw', 50)
        
        # 订阅最标准的机器人通用指令 Twist
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Bool, '/chassis/cmd_beep', self.beep_callback, 10)
        self.get_logger().info("底盘驱动封包器已就绪，等待 /cmd_vel 与警报指令")

    def beep_callback(self, msg: Bool):
        if msg.data:
            byte_frame = ChassisCodec.pack_beep(500) # 鸣笛 500ms
            out_msg = UInt8MultiArray()
            out_msg.data = list(byte_frame)
            self.tx_pub.publish(out_msg)

    def cmd_vel_callback(self, msg: Twist):
        # 转化平移与旋转请求为主板能懂的 0x12 字节段
        byte_frame = ChassisCodec.pack_velocity(msg.linear.x, msg.linear.y, msg.angular.z)
        out_msg = UInt8MultiArray()
        out_msg.data = list(byte_frame)
        self.tx_pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ChassisPackerNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

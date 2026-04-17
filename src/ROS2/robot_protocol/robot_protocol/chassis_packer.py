#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8MultiArray

from .protocol_stack import ChassisCodec


class ChassisPackerNode(Node):
    def __init__(self):
        super().__init__('chassis_packer_node')
        self.tx_pub = self.create_publisher(UInt8MultiArray, '/serial/tx_raw', 50)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Bool, '/chassis/cmd_beep', self.beep_callback, 10)
        self.get_logger().info('Chassis packer is ready.')

    def _publish_frame(self, frame):
        msg = UInt8MultiArray()
        msg.data = list(frame)
        self.tx_pub.publish(msg)

    def beep_callback(self, msg):
        if msg.data:
            self._publish_frame(ChassisCodec.pack_beep(500))

    def cmd_vel_callback(self, msg):
        self._publish_frame(ChassisCodec.pack_velocity(msg.linear.x, msg.linear.y, msg.angular.z))


def main(args=None):
    rclpy.init(args=args)
    node = ChassisPackerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

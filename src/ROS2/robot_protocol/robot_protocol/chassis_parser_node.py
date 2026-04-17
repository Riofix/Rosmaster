#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

from .protocol_stack import ChassisCodec, ChassisStreamParser


class ChassisParserNode(Node):
    def __init__(self):
        super().__init__('chassis_parser_node')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.parser = ChassisStreamParser()
        self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self.twist_pub = self.create_publisher(TwistStamped, '/chassis/twist', 20)
        self.create_subscription(UInt8MultiArray, '/serial/rx_raw', self.rx_callback, 100)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_motion_time = None
        self.get_logger().info('Chassis parser is ready.')

    def rx_callback(self, msg):
        for _device_id, func_code, payload in self.parser.feed(bytes(msg.data)):
            parsed = ChassisCodec.parse_rx(func_code, payload)
            if parsed['type'] == 'motion':
                self._handle_motion(parsed)

    def _handle_motion(self, parsed):
        now = self.get_clock().now()
        if self.last_motion_time is not None:
            dt = (now - self.last_motion_time).nanoseconds / 1e9
            dt = max(0.0, min(dt, 0.2))
            cos_yaw = math.cos(self.yaw)
            sin_yaw = math.sin(self.yaw)
            self.x += (parsed['vx'] * cos_yaw - parsed['vy'] * sin_yaw) * dt
            self.y += (parsed['vx'] * sin_yaw + parsed['vy'] * cos_yaw) * dt
            self.yaw += parsed['vth'] * dt
        self.last_motion_time = now

        twist_msg = TwistStamped()
        twist_msg.header.stamp = now.to_msg()
        twist_msg.header.frame_id = self.base_frame
        twist_msg.twist.linear.x = parsed['vx']
        twist_msg.twist.linear.y = parsed['vy']
        twist_msg.twist.angular.z = parsed['vth']
        self.twist_pub.publish(twist_msg)

        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom_msg.twist.twist.linear.x = parsed['vx']
        odom_msg.twist.twist.linear.y = parsed['vy']
        odom_msg.twist.twist.angular.z = parsed['vth']
        self.odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ChassisParserNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

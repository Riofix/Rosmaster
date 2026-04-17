#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import CraneState
from std_msgs.msg import UInt8, UInt8MultiArray

from .protocol_stack import CraneCodec, CraneStreamParser


class CraneParserNode(Node):
    def __init__(self):
        super().__init__('crane_parser_node')
        self.declare_parameter('crane_id', 'crane_left')
        self.declare_parameter('hoist_motor_addr', 2)
        self.declare_parameter('hoist_units_per_tick', 1.0)
        self.declare_parameter('hoist_positive_direction', 1)
        self.declare_parameter('outlet_open_angle', 100)

        self.crane_id = self.get_parameter('crane_id').value
        self.hoist_motor_addr = int(self.get_parameter('hoist_motor_addr').value)
        self.hoist_units_per_tick = float(self.get_parameter('hoist_units_per_tick').value)
        self.hoist_positive_direction = int(self.get_parameter('hoist_positive_direction').value)
        self.outlet_open_angle = int(self.get_parameter('outlet_open_angle').value)

        self.rx_parser = CraneStreamParser()
        self.tx_parser = CraneStreamParser()
        self.pending_hoist_target = None

        prefix = f'/{self.crane_id}'
        self.state_pub = self.create_publisher(CraneState, f'{prefix}/state', 20)
        self.done_pub = self.create_publisher(UInt8, f'{prefix}/motor_done', 20)
        self.error_pub = self.create_publisher(UInt8MultiArray, f'{prefix}/command_error', 20)

        self.create_subscription(UInt8MultiArray, f'{prefix}/tcp_rx_raw', self.rx_callback, 100)
        self.create_subscription(UInt8MultiArray, f'{prefix}/tcp_tx_raw', self.tx_callback, 100)

        self.state = CraneState()
        self.get_logger().info(f'Crane parser is ready for {self.crane_id}.')

    def _publish_state(self):
        self.state.header.stamp = self.get_clock().now().to_msg()
        self.state_pub.publish(self.state)

    def tx_callback(self, msg):
        for _header, func_code, payload in self.tx_parser.feed(bytes(msg.data)):
            parsed = CraneCodec.parse_tx(func_code, payload)
            if parsed['type'] == 'vacuum':
                self.state.vacuum_power = int(parsed['duty'])
            elif parsed['type'] == 'outlet_servo':
                self.state.is_outlet_open = parsed['angle'] >= self.outlet_open_angle
            elif parsed['type'] == 'emm_pos_ctrl' and parsed['addr'] == self.hoist_motor_addr:
                sign = 1.0 if parsed['dir'] == self.hoist_positive_direction else -1.0
                delta = sign * parsed['target_ticks'] * self.hoist_units_per_tick
                self.pending_hoist_target = self.state.hoist_depth + delta if parsed['is_rel'] else delta
            self._publish_state()

    def rx_callback(self, msg):
        for _header, func_code, payload in self.rx_parser.feed(bytes(msg.data)):
            parsed = CraneCodec.parse_rx(func_code, payload)
            event_type = parsed['type']
            if event_type == 'mpu_carriage':
                self.state.carriage_roll = float(parsed['roll'])
                self.state.carriage_pitch = float(parsed['pitch'])
                self.state.carriage_yaw = float(parsed['yaw'])
                self._publish_state()
            elif event_type == 'emm_odom':
                self.state.track_pos = float(parsed['pos'])
                self._publish_state()
            elif event_type == 'report_pos':
                if parsed['motor_addr'] == self.hoist_motor_addr and self.pending_hoist_target is not None:
                    self.state.hoist_depth = float(self.pending_hoist_target)
                    self.pending_hoist_target = None
                    self._publish_state()
                done_msg = UInt8()
                done_msg.data = int(parsed['motor_addr'])
                self.done_pub.publish(done_msg)
            elif event_type == 'ack_err':
                err = UInt8MultiArray()
                err.data = [int(parsed['motor_req']), int(parsed['err_code'])]
                self.error_pub.publish(err)


def main(args=None):
    rclpy.init(args=args)
    node = CraneParserNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from robot_interfaces.srv import CraneTrigger
from std_msgs.msg import Float32, Float32MultiArray, UInt8MultiArray

from .protocol_stack import CraneCodec


class CranePackerNode(Node):
    AXIS_TRACK = 1
    AXIS_HOIST = 2

    def __init__(self):
        super().__init__('crane_packer_node')
        self.declare_parameter('crane_id', 'crane_left')
        self.declare_parameter('hoist_motor_addr', 2)
        self.declare_parameter('step_acc', 50)

        self.crane_id = self.get_parameter('crane_id').value
        self.hoist_motor_addr = int(self.get_parameter('hoist_motor_addr').value)
        self.step_acc = int(self.get_parameter('step_acc').value)

        prefix = f'/{self.crane_id}'
        self.tx_pub = self.create_publisher(UInt8MultiArray, f'{prefix}/tcp_tx_raw', 50)
        self.create_service(CraneTrigger, f'{prefix}/trigger', self.trigger_cb)
        self.create_subscription(Float32MultiArray, f'{prefix}/debug_step', self.step_cb, 10)
        self.create_subscription(Float32, f'{prefix}/track_goal', self.track_goal_cb, 10)
        self.get_logger().info(f'Crane packer is ready for {self.crane_id}.')

    def _publish_frame(self, frame):
        msg = UInt8MultiArray()
        msg.data = list(frame)
        self.tx_pub.publish(msg)

    def track_goal_cb(self, msg):
        self._publish_frame(CraneCodec.pack_tracker_set_goal(msg.data))

    def step_cb(self, msg):
        if len(msg.data) < 4:
            self.get_logger().warning('debug_step expects [addr, dir, vel, ticks].')
            return
        addr = int(msg.data[0])
        direction = int(msg.data[1])
        vel = int(msg.data[2])
        target_ticks = int(msg.data[3])
        self._publish_frame(CraneCodec.pack_emm_pos_ctrl(addr, direction, vel, self.step_acc, target_ticks, True))

    def trigger_cb(self, request, response):
        if request.trigger_type == CraneTrigger.Request.TRIGGER_VACUUM:
            frame = CraneCodec.pack_vacuum_duty(request.target_value)
        elif request.trigger_type == CraneTrigger.Request.TRIGGER_OUTLET:
            frame = CraneCodec.pack_outlet_servo(1, request.target_value)
        else:
            response.success = False
            response.message = f'unknown trigger_type={request.trigger_type}'
            return response

        self._publish_frame(frame)
        response.success = True
        response.message = 'sent'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CranePackerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

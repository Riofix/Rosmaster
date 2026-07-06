#!/usr/bin/env python3
"""查看 world_state 全部数据"""
import rclpy, json
from rclpy.node import Node
from std_msgs.msg import String

class DumpAll(Node):
    def __init__(self):
        super().__init__('dump_all')
        self.create_subscription(String, '/world_state', self.cb, 10)
    def cb(self, msg):
        d = json.loads(msg.data)
        print(json.dumps(d, indent=2, ensure_ascii=False))
        raise SystemExit

rclpy.init()
rclpy.spin(DumpAll())

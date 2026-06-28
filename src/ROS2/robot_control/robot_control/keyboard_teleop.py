#!/usr/bin/env python3
"""
键盘遥控底盘 + 编码器测距
  W/S : 前进/后退  |  空格: 急停  |  R: 记录  |  Q: 退出
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
import sys
import termios
import tty
import threading
import select


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.cmd_pub = self.create_publisher(String, '/control_cmd', 10)
        self.create_subscription(String, '/robot_shadow_states', self.shadow_cb, 10)

        self.enc1 = self.enc3 = 0
        self.recorded1 = self.recorded3 = 0
        self.has_record = False
        self.running = True
        self.vx = 0

        self.get_logger().info("W/S=前后 空格=停 R=记录 Q=退出")

    def shadow_cb(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get("source") == "chassis":
                encs = data.get("state", {}).get("motor_encoder", [0,0,0,0])
                self.enc1 = encs[0] if len(encs) > 0 else 0
                self.enc3 = encs[2] if len(encs) > 2 else 0
        except: pass

    def send_speed(self, vx):
        self.vx = vx
        cmd = {"target": "chassis", "cmd_hex": 0x12, "sub_id": 0x01,
               "params": {"vx": vx, "vy": 0, "vz": 0}}
        self.cmd_pub.publish(String(data=json.dumps(cmd)))

    def _status_str(self):
        avg = (self.enc1 + self.enc3) // 2
        if self.has_record:
            d1 = self.enc1 - self.recorded1
            d3 = self.enc3 - self.recorded3
            return (f"E1:{self.enc1} E3:{self.enc3} Avg:{avg} Vx:{self.vx} | "
                    f"Δ  E1:{d1} E3:{d3} ΔAvg:{(d1+d3)//2}")
        return f"E1:{self.enc1} E3:{self.enc3} Avg:{avg} Vx:{self.vx} | 按R记录"

    def key_loop(self):
        tty_path = "/dev/tty"
        if not os.path.exists(tty_path):
            tty_path = "/dev/stdin"
        try:
            fd = os.open(tty_path, os.O_RDONLY | os.O_NONBLOCK)
        except OSError:
            self.get_logger().error(f"无法打开 {tty_path}, 键盘不可用")
            return

        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)

        while self.running and rclpy.ok():
            r, _, _ = select.select([fd], [], [], 0.2)
            if r:
                key = os.read(fd, 1).decode('utf-8', errors='ignore').lower()
                if key == 'w':      self.send_speed(200)
                elif key == 's':    self.send_speed(-200)
                elif key == ' ':    self.send_speed(0)
                elif key == 'r':
                    self.recorded1 = self.enc1; self.recorded3 = self.enc3
                    self.has_record = True
                    self.get_logger().info(f"◎ 记录起点: E1={self.enc1} E3={self.enc3}")
                elif key == 'q':
                    self.send_speed(0); self.running = False
                    self.get_logger().info("退出")
            self.get_logger().info(self._status_str())

        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        os.close(fd)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    t = threading.Thread(target=node.key_loop, daemon=True)
    t.start()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

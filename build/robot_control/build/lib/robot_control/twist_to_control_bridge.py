#!/usr/bin/env python3
"""
Twist → /control_cmd 桥接节点
订阅 /cmd_vel (geometry_msgs/Twist)，转换为底盘 JSON 协议发到 /control_cmd
配合官方 teleop_twist_keyboard 使用
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json


class TwistToControlBridge(Node):
    def __init__(self):
        super().__init__('twist_to_control_bridge')

        # 底盘类型，默认 0x07 = CAR_FOURWHEEL_SAME_DIR
        self.car_type = self.declare_parameter('car_type', 7).value

        # Twist 速度换算系数
        # linear.x: m/s → mm/s,  ×1000
        # angular.z: rad/s → chassis Vz, ×2000
        self.linear_scale = self.declare_parameter('linear_scale', 1000).value
        self.angular_scale = self.declare_parameter('angular_scale', 2000).value

        # 限幅
        self.max_vx = self.declare_parameter('max_vx', 1000).value
        self.max_vz = self.declare_parameter('max_vz', 3000).value

        self.sub = self.create_subscription(Twist, '/cmd_vel', self.twist_cb, 10)
        self.pub = self.create_publisher(String, '/control_cmd', 10)

        self.get_logger().info(f'Twist→Control 桥接就绪, car_type=0x{self.car_type:02X}')
        self.get_logger().info(f'  linear_scale={self.linear_scale}, angular_scale={self.angular_scale}')
        self.get_logger().info(f'  max Vx={self.max_vx}, max Vz={self.max_vz}')

    def twist_cb(self, msg: Twist):
        # 转换: m/s → 整数 mm/s
        vx = int(msg.linear.x * self.linear_scale)
        vy = int(msg.linear.y * self.linear_scale)  # 四轮同向车忽略，协议仍可传
        vz = int(msg.angular.z * self.angular_scale)

        # 限幅
        vx = max(min(vx, self.max_vx), -self.max_vx)
        vz = max(min(vz, self.max_vz), -self.max_vz)

        # 四轮同向无横向移动，vy 清零
        payload = {
            "target": "chassis",
            "sub_id": self.car_type,
            "cmd_hex": 0x12,
            "params": {"vx": vx, "vy": 0, "vz": vz}
        }

        msg_out = String()
        msg_out.data = json.dumps(payload)
        self.pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = TwistToControlBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

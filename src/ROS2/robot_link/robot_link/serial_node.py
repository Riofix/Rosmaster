#!/usr/import/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import serial
import threading

class SerialLinkNode(Node):
    """
    纯粹的链路层物理节点 (根据“盲操作”原则重构)
    完全不处理任何通信协议规则(剥离了各种 0xFF 校验)，只负责：
    1. 从串口读走所有能读到的字节块，然后原封不动地发往 /serial/rx_raw
    2. 接收来自 /serial/tx_raw 的字节块，原封不动地写入串口
    """
    def __init__(self):
        super().__init__('serial_link_node')
        
        self.declare_parameter('port', '/dev/rosmaster')
        self.declare_parameter('baudrate', 115200)
        
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.is_running = False

        # 发布接收到的原始字节数组
        self.rx_pub = self.create_publisher(UInt8MultiArray, '/serial/rx_raw', 100)
        # 订阅需要发送的原始字节数组
        self.tx_sub = self.create_subscription(UInt8MultiArray, '/serial/tx_raw', self.tx_callback, 100)

        # 尝试连接硬件
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.01)
            self.is_running = True
            self.get_logger().info(f"串口物理链路成功建立: {self.port} @ {self.baudrate}")
            
            # 开启后台无脑读取线程
            self.read_thread = threading.Thread(target=self._physical_read_loop, daemon=True)
            self.read_thread.start()
        except Exception as e:
            self.get_logger().error(f"严重物理故障！无法打开串口 {self.port} : {e}")

    def tx_callback(self, msg: UInt8MultiArray):
        """链路层盲发：来什么数据就往物理层灌什么数据"""
        if self.is_running and self.serial.is_open:
            try:
                self.serial.write(bytes(msg.data))
            except Exception as e:
                self.get_logger().error(f"串口发送遭遇错误: {e}")

    def _physical_read_loop(self):
        """链路层盲收：不解析协议，只按 chunk 块打包上推给协议层"""
        while self.is_running and rclpy.ok():
            try:
                count = self.serial.in_waiting
                if count > 0:
                    raw_bytes = self.serial.read(count)
                    
                    # 打包推上 ROS 网络 (丢给 robot_protocol 让它痛苦去)
                    msg = UInt8MultiArray()
                    msg.data = list(raw_bytes)
                    self.rx_pub.publish(msg)
                else:
                    # 避免 100% CPU，由于串口有波特率限制，睡一点点完全可以
                    import time
                    time.sleep(0.002)
            except Exception as e:
                self.get_logger().error(f"串口物理层读取断开: {e}")
                self.is_running = False
                break

    def destroy_node(self):
        self.is_running = False
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialLinkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

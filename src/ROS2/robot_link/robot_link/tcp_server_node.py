#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from robot_interfaces.msg import DeviceState
import asyncio
import threading

class TcpLinkServerNode(Node):
    """
    负责 ESP8266 接入的 TCP 物理链路服务器
    功能：
    1. 监听 8080 端口，接受 STM32 吊具下位机长连接
    2. 基于静态 IP 白名单进行设备名称映射 (Blind Operation 核心机制)
    3. 动态发布每个吊具独立的收发话题 (例如 /crane_left/tcp_rx_raw)
    """
    def __init__(self):
        super().__init__('tcp_server_link_node')
        
        self.declare_parameter('port', 8080)
        self.bind_port = self.get_parameter('port').value
        
        # 静态 IP 映射表 (在这里彻底隐藏下游 IP，将其转换为对上层友好的 namespace)
        self.ip_mapping = {
            "192.168.1.101": "crane_left",
            "192.168.1.102": "crane_middle",
            "192.168.1.103": "crane_right"
        }

        # 保存活动客户端流 (注意: 不能命名为 clients，与 rclpy Node 内部保留属性冲突)
        self._tcp_clients = {}  # {device_name: StreamWriter}
        
        # 为每个设备动态注册 Publisher 和 Subscriber 
        self.rx_pubs = {}
        self.tx_subs = {}

        # 状态播报器
        self.status_pub = self.create_publisher(DeviceState, '/device_status', 10)

        # 启动 Asyncio 线程
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._start_async_server, daemon=True)
        self._thread.start()

    def _start_async_server(self):
        asyncio.set_event_loop(self._loop)
        coro = asyncio.start_server(self._handle_client, '0.0.0.0', self.bind_port)
        server = self._loop.run_until_complete(coro)
        self.get_logger().info(f"TCP 链路服务器已启动，正监听端口: {self.bind_port}")
        self._loop.run_forever()

    async def _handle_client(self, reader, writer):
        addr = writer.get_extra_info('peername')
        ip = addr[0]
        
        # 进行盲操作核心转换
        if ip not in self.ip_mapping:
            self.get_logger().warn(f"拒绝未知设备连接: {ip}")
            writer.close()
            return
            
        device_name = self.ip_mapping[ip]
        self._tcp_clients[device_name] = writer
        self.get_logger().info(f"设备上线: [{device_name}] IP:{ip}")
        
        # 发布上线状态
        self._publish_status(device_name, ip, DeviceState.STATUS_IDLE)

        # 动态创建 ROS 桥接通道 (如果没有创建过)
        if device_name not in self.rx_pubs:
            self.rx_pubs[device_name] = self.create_publisher(UInt8MultiArray, f'/{device_name}/tcp_rx_raw', 50)
            def make_tx_callback(dev_id):
                def tx_callback(msg: UInt8MultiArray):
                    if dev_id in self._tcp_clients:
                        w = self._tcp_clients[dev_id]
                        # 在 asyncio 线程里安全地写入数据
                        asyncio.run_coroutine_threadsafe(self._async_write(w, bytes(msg.data)), self._loop)
                return tx_callback
            self.tx_subs[device_name] = self.create_subscription(
                UInt8MultiArray, f'/{device_name}/tcp_tx_raw', make_tx_callback(device_name), 50)

        # 进入物理读取死循环
        while rclpy.ok():
            try:
                data = await reader.read(1024)
                if not data:
                    break
                msg = UInt8MultiArray()
                msg.data = list(data)
                self.rx_pubs[device_name].publish(msg)
            except Exception as e:
                self.get_logger().error(f"设备读取异常 {device_name}: {e}")
                break
                
        # 设备下线处理
        self.get_logger().warn(f"设备下线: [{device_name}]")
        self._publish_status(device_name, ip, DeviceState.STATUS_OFFLINE)
        if device_name in self._tcp_clients:
            del self._tcp_clients[device_name]
        writer.close()

    async def _async_write(self, writer, data: bytes):
        """在 asyncio 线程中安全写入并 flush 发送缓冲区"""
        try:
            writer.write(data)
            await writer.drain()
        except Exception as e:
            self.get_logger().error(f"TCP 写入异常: {e}")

    def _publish_status(self, name, ip, status):
        msg = DeviceState()
        msg.device_name = name
        msg.ip_address = ip
        msg.current_status = status
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TcpLinkServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

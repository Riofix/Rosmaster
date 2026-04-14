#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import subprocess

class HotspotNode(Node):
    """
    负责在 RDK X5 每次开机/启动 ROS 节点时，自动建立 WiFi 局域网热点。
    保证底盘上的所有下位机（ESP8266）无论环境怎么变，都有固定的网络能够回连。
    """
    def __init__(self):
        super().__init__('hotspot_node')
        
        # 您可以在后续 launch 文件中动态修改这些配置
        self.declare_parameter('ssid', 'RDK_CRANE')
        self.declare_parameter('password', '12345678')
        self.declare_parameter('interface', 'wlan0')
        
        self.ssid = self.get_parameter('ssid').value
        self.password = self.get_parameter('password').value
        self.interface = self.get_parameter('interface').value
        
        self.enable_hotspot()

    def enable_hotspot(self):
        self.get_logger().info(f"正在尝试通过节点开启局域物理热点: {self.ssid} ...")
        try:
            # 在 Ubuntu 系统下，通常使用 nmcli 最稳妥
            cmd = [
                'nmcli', 'device', 'wifi', 'hotspot', 
                'ifname', self.interface, 
                'ssid', self.ssid, 
                'password', self.password
            ]
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                self.get_logger().info(f"热点 [{self.ssid}] 已成功建立！TCP服务器已具备底层网络通讯能力。")
            else:
                self.get_logger().warn(f"热点开启提示 (如果已经有其他程序开启了该热点可能会报错):\n{result.stderr}")
                
        except Exception as e:
            self.get_logger().error(f"热点启动失败，请检查 RDK X5 系统是否安装了网络管理器: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = HotspotNode()
    
    # 因为只需要启动一次就完成历史使命了，所以可以直接销毁节点，
    # 也可以留在后台待命处理其他网络事务
    rclpy.spin_once(node, timeout_sec=2.0)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

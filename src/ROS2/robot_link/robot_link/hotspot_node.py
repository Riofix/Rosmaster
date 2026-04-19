import rclpy
from rclpy.node import Node
import subprocess

class HotspotNode(Node):
    def __init__(self):
        # 节点名称正式定为 HotspotNode
        super().__init__('HotspotNode')
        
        # 声明 ROS 2 参数 (解耦设计)
        self.declare_parameter('ssid', 'Digua')
        self.declare_parameter('password', '12345678')
        self.declare_parameter('interface', 'wlan0')
        self.declare_parameter('ip_address', '192.168.1.1/24')

        # 执行网络配置逻辑
        self.configure_network()

    def configure_network(self):
        ssid = self.get_parameter('ssid').get_parameter_value().string_value
        password = self.get_parameter('password').get_parameter_value().string_value
        iface = self.get_parameter('interface').get_parameter_value().string_value
        ip_addr = self.get_parameter('ip_address').get_parameter_value().string_value

        self.get_logger().info(f'Configuring WiFi Hotspot: {ssid}...')

        try:
            # 1. 删除可能存在的旧连接，确保干净的配置环境
            subprocess.run(['nmcli', 'connection', 'delete', 'Hotspot'], capture_output=True)

            # 2. 创建持久化的无线热点
            subprocess.run([
                'nmcli', 'device', 'wifi', 'hotspot',
                'ssid', ssid,
                'password', password,
                'ifname', iface
            ], check=True, capture_output=True)

            # 3. 配置 IPv4 静态地址段，供后续 TCP 链路使用
            subprocess.run([
                'nmcli', 'connection', 'modify', 'Hotspot',
                'ipv4.method', 'shared',
                'ipv4.addresses', ip_addr
            ], check=True, capture_output=True)

            # 4. 激活网络连接
            subprocess.run(['nmcli', 'connection', 'up', 'Hotspot'], check=True, capture_output=True)
            
            self.get_logger().info('Hotspot setup completed successfully.')
            
        except subprocess.CalledProcessError as e:
            error_output = e.stderr.decode() if e.stderr else str(e)
            self.get_logger().error(f'Failed to configure Hotspot: {error_output}')

def main(args=None):
    rclpy.init(args=args)
    
    # 实例化节点
    node = HotspotNode()
    
    # 执行完逻辑后直接销毁，不驻留内存
    node.get_logger().info('HotspotNode task finished. Shutting down.')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
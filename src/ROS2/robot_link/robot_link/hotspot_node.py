import rclpy
from rclpy.node import Node
import subprocess
import sys

class HotspotSetupNode(Node):
    def __init__(self):
        super().__init__('hotspot_setup_node')
        
        # 1. 声明参数，方便以后通过 launch 文件或命令行修改
        self.declare_parameter('ssid', 'Digua')
        self.declare_parameter('password', '12345678')
        self.declare_parameter('interface', 'wlan0')
        self.declare_parameter('ip_address', '10.42.0.1/24')

    def activate_hotspot(self):
        # 获取参数值
        ssid = self.get_parameter('ssid').value
        password = self.get_parameter('password').value
        iface = self.get_parameter('interface').value
        ip_addr = self.get_parameter('ip_address').value

        self.get_logger().info(f'正在通过 nmcli 配置热点: {ssid}...')

        try:
            # 步骤 A: 删除旧连接（防止配置冲突）
            # 提示：nmcli 配置是持久化的，删除旧的可以确保应用新参数
            subprocess.run(['sudo', 'nmcli', 'connection', 'delete', ssid], capture_output=True)

            # 步骤 B: 创建热点连接
            # 提示：Mode 设置为 Hotspot，Band 通常为 bg (2.4GHz)
            create_cmd = [
                'sudo', 'nmcli', 'device', 'wifi', 'hotspot',
                'con-name', ssid,
                'ssid', ssid,
                'password', password,
                'ifname', iface
            ]
            subprocess.run(create_cmd, check=True, capture_output=True)

            # 步骤 C: 固定网关 IP (设置为 10.42.0.1)
            # 提示：ipv4.method 必须为 shared 才能启用 DHCP 分配 IP 给其他设备
            modify_cmd = [
                'sudo', 'nmcli', 'connection', 'modify', ssid,
                'ipv4.method', 'shared',
                'ipv4.addresses', ip_addr
            ]
            subprocess.run(modify_cmd, check=True, capture_output=True)

            # 步骤 D: 激活连接
            subprocess.run(['sudo', 'nmcli', 'connection', 'up', ssid], check=True, capture_output=True)
            
            self.get_logger().info(f'成功！热点 "{ssid}" 已在后台运行，网关: {ip_addr.split("/")[0]}')
            return True
            
        except subprocess.CalledProcessError as e:
            error_info = e.stderr.decode() if e.stderr else str(e)
            self.get_logger().error(f'配置失败: {error_info}')
            return False

def main(args=None):
    rclpy.init(args=args)
    node = HotspotSetupNode()
    
    # 执行开启逻辑
    success = node.activate_hotspot()
    
    # 任务完成后立即关闭节点，释放内存
    node.get_logger().info('任务完成，正在退出 ROS 2 节点...')
    node.destroy_node()
    rclpy.shutdown()
    
    # 根据结果退出进程
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()
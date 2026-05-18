import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import socket
import threading

class TcpServerNode(Node):
    def __init__(self):
        super().__init__('TcpServerNode')
        
        # 1. 声明参数
        self.declare_parameter('port', 8080)
        self.declare_parameter('ip_left', '192.168.1.101')
        self.declare_parameter('ip_mid', '192.168.1.102')
        self.declare_parameter('ip_right', '192.168.1.103')

        self.port = self.get_parameter('port').value
        self.ip_map = {
            self.get_parameter('ip_left').value: 'handle_left',
            self.get_parameter('ip_mid').value: 'handle_mid',
            self.get_parameter('ip_right').value: 'handle_right'
        }

        # 存储已连接的客户端 { 'handle_left': socket_obj }
        self.client_sockets = {}

        # 2. 发布者：发布接收到的原始字节，增加一个来源标识字段（通过自定义消息或在数据前加前缀，
        # 这里为了简单，我们发布包含设备信息的 UInt8MultiArray）
        self.rx_pub = self.create_publisher(UInt8MultiArray, '/tcp_rx_raw', 10)

        # 3. 订阅者：接收要发送给 TCP 客户端的原始字节
        # 格式约定：数据包的第一个字节作为目标标识（0:left, 1:mid, 2:right），后续为实际负载
        self.create_subscription(UInt8MultiArray, '/tcp_tx_raw', self.tx_callback, 10)

        # 4. 启动服务器线程
        self.server_thread = threading.Thread(target=self.run_server, daemon=True)
        self.server_thread.start()
        
        self.get_logger().info(f'TCP Server Node started on port {self.port}')

    def run_server(self):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            server_socket.bind(('0.0.0.0', self.port))
            server_socket.listen(5)
            
            while rclpy.ok():
                client_sock, addr = server_socket.accept()
                client_ip = addr[0]
                device_name = self.ip_map.get(client_ip)
                
                if device_name:
                    self.get_logger().info(f'{device_name} connected from {client_ip}')
                    self.client_sockets[device_name] = client_sock
                    threading.Thread(target=self.receive_loop, args=(client_sock, device_name), daemon=True).start()
                else:
                    self.get_logger().warn(f'Unknown IP {client_ip} rejected.')
                    client_sock.close()
        except Exception as e:
            self.get_logger().error(f'Server error: {e}')
        finally:
            server_socket.close()

    def receive_loop(self, client_sock, device_name):
        """接收数据并发布到 /tcp_rx_raw"""
        # 定义标识：left=0, mid=1, right=2
        device_id = list(self.ip_map.values()).index(device_name)
        
        while rclpy.ok():
            try:
                data = client_sock.recv(1024)
                if not data:
                    break
                
                msg = UInt8MultiArray()
                # 协议解耦：在原始数据前插入一个字节的设备 ID，方便解析节点区分来源
                msg.data = [device_id] + list(data)
                self.rx_pub.publish(msg)
            except:
                break
        
        self.get_logger().warn(f'{device_name} disconnected.')
        if device_name in self.client_sockets:
            del self.client_sockets[device_name]
        client_sock.close()

    def tx_callback(self, msg):
        """
        从 /tcp_tx_raw 接收数据并分发给对应的 TCP 客户端
        msg.data 格式: [device_id, raw_byte1, raw_byte2, ...]
        """
        if len(msg.data) < 2:
            return
            
        device_id = msg.data[0]
        payload = bytes(msg.data[1:])
        
        device_names = list(self.ip_map.values())
        if device_id < len(device_names):
            target_name = device_names[device_id]
            if target_name in self.client_sockets:
                try:
                    self.client_sockets[target_name].send(payload)
                except Exception as e:
                    self.get_logger().error(f'TCP send error to {target_name}: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TcpServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

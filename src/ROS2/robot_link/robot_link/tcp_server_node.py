import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import socket
import threading

class TcpServerNode(Node):
    def __init__(self):
        super().__init__('TcpServerNode')
        
        # 1. 声明参数
        self.declare_parameter('port', 3456)
        self.port = self.get_parameter('port').value

        # 存储已验证身份的客户端 { 'left': socket_obj, 'mid': socket_obj, ... }
        self.clients = {}
        # 映射表，用于给 ROS 2 消息打标：0:left, 1:mid, 2:right
        self.device_id_map = {'left': 0, 'mid': 1, 'right': 2}

        # 2. 发布者：发送给 ROS 2 其他节点
        self.rx_pub = self.create_publisher(UInt8MultiArray, '/tcp_rx_raw', 10)

        # 3. 订阅者：接收要发给 STM32 的数据
        # 格式：[device_id, data1, data2...]
        self.tx_sub = self.create_subscription(UInt8MultiArray, '/tcp_tx_raw', self.tx_callback, 10)

        # 4. 启动服务器线程
        self.server_thread = threading.Thread(target=self.run_server, daemon=True)
        self.server_thread.start()
        
        self.get_logger().info(f'TCP 服务器已启动，监听端口: {self.port}')

    def run_server(self):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            server_socket.bind(('0.0.0.0', self.port))
            server_socket.listen(5)
            
            while rclpy.ok():
                client_sock, addr = server_socket.accept()
                # 开启新线程去处理身份验证，不阻塞主循环
                threading.Thread(target=self.auth_and_receive, args=(client_sock, addr), daemon=True).start()
        except Exception as e:
            self.get_logger().error(f'服务器异常: {e}')
        finally:
            server_socket.close()

    def auth_and_receive(self, client_sock, addr):
        """身份验证与数据接收"""
        device_name = None
        try:
            # --- 第一阶段：身份验证 ---
            # 设置 10 秒超时，如果 10 秒内 STM32 没发 ID 包，就断开连接
            client_sock.settimeout(10.0)
            data = client_sock.recv(1024).decode('utf-8').strip()
            
            if data.startswith("ID:"):
                device_name = data.split(":")[1]
                if device_name in self.device_id_map:
                    self.get_logger().info(f'设备身份确认: {device_name} (来自 {addr[0]})')
                    self.clients[device_name] = client_sock
                else:
                    self.get_logger().error(f'未知设备 ID: {device_name}，断开连接。')
                    client_sock.close()
                    return
            else:
                self.get_logger().warn(f'非法握手数据: {data}，断开连接。')
                client_sock.close()
                return

            # --- 第二阶段：正常接收数据 ---
            # 验证通过后，移除超时限制，进入正常接收循环
            client_sock.settimeout(None)
            device_id = self.device_id_map[device_name]
            
            while rclpy.ok():
                raw_data = client_sock.recv(2048)
                if not raw_data:
                    break
                
                # 发布到 ROS 2 话题
                msg = UInt8MultiArray()
                msg.data = [device_id] + list(raw_data)
                self.rx_pub.publish(msg)
                
        except (socket.timeout, Exception) as e:
            self.get_logger().warn(f'连接处理异常 ({addr[0]}): {e}')
        finally:
            if device_name and device_name in self.clients:
                del self.clients[device_name]
            client_sock.close()
            self.get_logger().info(f'设备 {device_name if device_name else addr[0]} 已断开')

    def tx_callback(self, msg):
        """
        处理外发数据
        msg.data 格式: [device_id, byte1, byte2...]
        """
        if len(msg.data) < 2: return
        
        target_id = msg.data[0]
        payload = bytes(msg.data[1:])
        
        # 查找 ID 对应的名称
        for name, d_id in self.device_id_map.items():
            if d_id == target_id:
                if name in self.clients:
                    try:
                        self.clients[name].send(payload)
                    except Exception as e:
                        self.get_logger().error(f'向 {name} 发送数据失败: {e}')
                break

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
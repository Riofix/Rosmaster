import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import serial
import threading

class SerialNode(Node):
    def __init__(self):
        super().__init__('SerialNode')
        
        # 1. 声明参数
        self.declare_parameter('port', '/dev/rosmaster')
        self.declare_parameter('baudrate', 115200)

        self.port_name = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value

        # 2. 发布者：串口接收到的数据发布到 /serial_rx_raw
        self.raw_pub = self.create_publisher(UInt8MultiArray, '/serial_rx_raw', 10)

        # 3. 订阅者：其他节点发给串口的数据从 /serial_tx_raw 接收
        self.create_subscription(UInt8MultiArray, '/serial_tx_raw', self.tx_callback, 10)

        # 4. 初始化串口 (严格匹配：8位数据位，无校验位，1位停止位)
        try:
            self.ser = serial.Serial(
                port=self.port_name,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,     # 数据位 8位
                parity=serial.PARITY_NONE,     # 无校验位
                stopbits=serial.STOPBITS_ONE,  # 停止位 1位
                timeout=0.1
            )
            self.get_logger().info(f'Serial port {self.port_name} opened (115200, 8N1).')
        except Exception as e:
            self.get_logger().error(f'Failed to open {self.port_name}: {e}')
            return

        # 5. 开启读取线程
        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

    def read_loop(self):
        """持续循环读取"""
        while rclpy.ok():
            if self.ser.is_open and self.ser.in_waiting > 0:
                try:
                    raw_bytes = self.ser.read(self.ser.in_waiting)
                    msg = UInt8MultiArray()
                    msg.data = list(raw_bytes)
                    self.raw_pub.publish(msg)
                except Exception as e:
                    self.get_logger().error(f'Read error: {e}')

    def tx_callback(self, msg):
        """发送回调"""
        if self.ser.is_open:
            try:
                self.ser.write(bytes(msg.data))
            except Exception as e:
                self.get_logger().error(f'Write error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
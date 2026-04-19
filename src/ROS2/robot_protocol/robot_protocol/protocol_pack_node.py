import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray
import struct
import json

class ProtocolPackNode(Node):
    def __init__(self):
        super().__init__('protocol_pack_node')
        
        # 1. 订阅控制层语义指令
        self.create_subscription(String, '/control_cmd', self.control_cb, 10)
        
        # 2. 发布到 Link 层的话题
        self.tcp_tx_pub = self.create_publisher(UInt8MultiArray, '/tcp_tx_raw', 10)
        self.ser_tx_pub = self.create_publisher(UInt8MultiArray, '/serial_tx_raw', 10)

        # 3. 设备路由映射 (0-2 为 TCP 客户端，3 为串口底盘)
        self.target_to_sid = {
            "handle_left": 0,
            "handle_mid":  1,
            "handle_right": 2,
            "chassis":     3
        }

        self.get_logger().info("Protocol Packing Node Online. Dedicated to Downlink Commands.")

    def control_cb(self, msg):
        """
        核心处理：接收 Control 层 JSON，封包为二进制流
        输入: {"target": "...", "sub_id": 0x01, "cmd_hex": 0x10, "params": {...}}
        """
        try:
            task = json.loads(msg.data)
            target = task.get("target")
            sub_id = task.get("sub_id")
            cmd_hex = task.get("cmd_hex")
            params = task.get("params", {})

            # --- 步骤 1: 构建数据段 (Data Section) ---
            # 协议约定：data[0] 为 cmd_hex，data[1] 为执行器 ID
            data_payload = bytearray([cmd_hex, sub_id])
            
            # 步骤 1.1: 根据参数名进行小端序打包 (struct.pack)
            # 这里是扩展点，根据下位机不同 CMD 的需求增加打包逻辑
            if "pos" in params:
                data_payload.extend(struct.pack('<h', int(params["pos"])))
            elif "duty" in params:
                data_payload.extend(struct.pack('<B', int(params["duty"])))
            elif "angle" in params:
                data_payload.extend(struct.pack('<B', int(params["angle"])))

            # --- 步骤 2: 生成标准 FF FC 协议包 ---
            packet = self.build_ff_fc_packet(data_payload)

            # --- 步骤 3: 路由适配 ---
            sid = self.target_to_sid.get(target)
            if sid is None:
                self.get_logger().error(f"Unknown target: {target}")
                return

            # 根据 Link 层需求，数据包格式为 [sid, raw_byte1, raw_byte2...]
            out_msg = UInt8MultiArray()
            out_msg.data = [sid] + list(packet)

            # --- 步骤 4: 分发话题 ---
            if sid == 3: # 串口
                self.ser_tx_pub.publish(out_msg)
            else:       # TCP
                self.tcp_tx_pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Packing Logic Error: {e}")

    def build_ff_fc_packet(self, data_section):
        """
        封包规则实现：
        FF FC (包头) + LEN (长度位) + DATA (内容) + CHECKSUM (校验位)
        """
        packet = bytearray([0xFF, 0xFC])
        
        # 长度计算：LEN位(1) + DATA段长度 + CHECKSUM位(1)
        length = len(data_section) + 2
        packet.append(length)
        
        # 填充内容
        packet.extend(data_section)
        
        # 校验位：从长度位（下标2）开始累加，对 256 取余
        checksum = sum(packet[2:]) % 256
        packet.append(checksum)
        
        return packet

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ProtocolPackNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
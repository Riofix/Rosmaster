import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray
import struct
import json

class ProtocolPackNode(Node):
    def __init__(self):
        super().__init__('protocol_pack_node')
        
        self.create_subscription(String, '/control_cmd', self.control_cb, 10)
        self.tcp_tx_pub = self.create_publisher(UInt8MultiArray, '/tcp_tx_raw', 10)
        self.ser_tx_pub = self.create_publisher(UInt8MultiArray, '/serial_tx_raw', 10)

        self.target_to_sid = {
            "handle_left": 0, "handle_mid": 1, "handle_right": 2, "chassis": 3
        }
        self.get_logger().info("Protocol Packing Node Online.")

    def control_cb(self, msg):
        try:
            task = json.loads(msg.data)
            target = task.get("target")
            sub_id = task.get("sub_id")
            cmd_hex = task.get("cmd_hex")
            params = task.get("params", {})
            if sub_id is None:
                sub_id = 0

            # --- 步骤 1: 构建数据段 (Data Section) ---
            data_payload = bytearray([cmd_hex, sub_id])
            if target != "chassis":
                data_payload = self.build_handle_payload(cmd_hex, sub_id, params)
            elif "vx" not in params:
                pass
            
            # --- 【新增】底盘速度指令适配 ---
            if target == "chassis" and "vx" in params:
                # 解析出底盘期望的速度，使用 <hhh 压入 6 个字节
                vx = int(params.get("vx", 0))
                vy = int(params.get("vy", 0))
                vz = int(params.get("vz", 0))
                data_payload.extend(struct.pack('<hhh', vx, vy, vz))
                
            # 其余保持不变
            elif target == "chassis" and "pos" in params:
                data_payload.extend(struct.pack('<h', int(params["pos"])))
            elif target == "chassis" and "duty" in params:
                data_payload.extend(struct.pack('<B', int(params["duty"])))
            elif target == "chassis" and "angle" in params:
                data_payload.extend(struct.pack('<B', int(params["angle"])))

            # --- 步骤 2: 封包 ---
            packet = self.build_ff_fc_packet(data_payload)

            # --- 步骤 3: 路由分发 ---
            sid = self.target_to_sid.get(target)
            if sid is None: return

            out_msg = UInt8MultiArray()
            out_msg.data = [sid] + list(packet)

            if sid == 3:
                self.ser_tx_pub.publish(out_msg)
            else:
                self.tcp_tx_pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Packing Logic Error: {e}")

    def build_handle_payload(self, cmd_hex, sub_id, params):
        motor_addr = int(params.get("motor_addr", sub_id if sub_id is not None else 1))
        snf = 1 if params.get("sync", False) else 0

        if cmd_hex == 0x60:
            enable = 1 if params.get("enable", True) else 0
            return bytearray([cmd_hex, motor_addr, enable, snf])

        if cmd_hex == 0x61:
            direction = int(params.get("dir", params.get("direction", 0)))
            speed = int(params.get("speed", params.get("vel", 0)))
            acc = int(params.get("acc", 0))
            return bytearray([
                cmd_hex, motor_addr, direction,
                (speed >> 8) & 0xFF, speed & 0xFF,
                acc & 0xFF, snf
            ])

        if cmd_hex == 0x62:
            direction = int(params.get("dir", params.get("direction", 0)))
            speed = int(params.get("speed", params.get("vel", 0)))
            acc = int(params.get("acc", 0))
            pulses = int(params.get("pos", params.get("pulses", params.get("clk", 0))))
            absolute_flag = 0 if params.get("relative", False) else 1
            return bytearray([
                cmd_hex, motor_addr, direction,
                (speed >> 8) & 0xFF, speed & 0xFF,
                acc & 0xFF,
                (pulses >> 24) & 0xFF,
                (pulses >> 16) & 0xFF,
                (pulses >> 8) & 0xFF,
                pulses & 0xFF,
                absolute_flag, snf
            ])

        if cmd_hex == 0x63:
            return bytearray([cmd_hex, motor_addr, snf])

        if cmd_hex == 0x6C:
            channel = int(params.get("channel", sub_id if sub_id is not None else 0))
            angle = int(params.get("angle", params.get("pos", 0)))
            return bytearray([cmd_hex, channel, angle & 0xFF])

        if cmd_hex == 0x6D:
            duty = int(params.get("duty", params.get("speed", sub_id if sub_id is not None else 0)))
            return bytearray([cmd_hex, duty & 0xFF])

        if cmd_hex == 0x6E:
            return bytearray([cmd_hex])

        raise ValueError(f"Unsupported handle cmd_hex: {cmd_hex}")

    def build_ff_fc_packet(self, data_section):
        packet = bytearray([0xFF, 0xFC])
        length = len(data_section) + 2
        packet.append(length)
        packet.extend(data_section)
        checksum = sum(packet[2:]) % 256
        packet.append(checksum)
        return packet

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ProtocolPackNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

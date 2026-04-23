import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray
import struct
import json

class ProtocolNode(Node):
    def __init__(self):
        super().__init__('ProtocolNode')
        
        # ======================== 影子结构隔离 ========================
        
        # 1. Handle (TCP) 的专属状态结构
        self.handle_states = {
            name: {
                "motor_odom": [0, 0],
                "mpu": {"r": 0.0, "p": 0.0, "y": 0.0},
                "bldc_duty": 0,
                "servo_angle": 0,
                "color_id": 0,
                "track_arrived": False, # 指示：抓手在水平横梁轨道上到位
                "action_done": False,   # 指示：抓手垂直升降/抓取流程完成 (注意这里补了逗号)
                "task_id": 0            # 预留，由 Control 层通过特定方式触发更新
            } for name in ["handle_left", "handle_mid", "handle_right"]
        }

        # 2. Chassis (串口底盘) 的专属状态结构
        self.chassis_state = {
            "speed": {"vx": 0, "vy": 0, "vz": 0},
            "voltage": 0.0,
            "imu_raw": {
                "gyro": [0.0, 0.0, 0.0],
                "acc": [0.0, 0.0, 0.0],
                "mag": [0.0, 0.0, 0.0]
            },
            "imu_euler": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            "motor_encoder": [0, 0, 0, 0],
            "arrival_done": False,      # 预留，由 Control 层触发更新 
            "task_id": 0                # 预留，由 Control 层触发更新
        }

        # ======================== 解析表驱动隔离 ========================
        self.handle_cmd_table = {
            0x64: self.parse_handle_mpu,
            0x65: self.parse_handle_odom,
            0x60: self.parse_handle_ack,
            0x62: self.parse_handle_bldc,
            0x66: self.parse_handle_color,
        }

        self.chassis_cmd_table = {
            0x0A: self.parse_chassis_status,
            0x0E: self.parse_chassis_imu_raw,
            0x0C: self.parse_chassis_imu_euler,
            0x0D: self.parse_chassis_odom,
        }

        # ======================== 基础配置 ========================
        self.bufs = {i: bytearray() for i in range(4)} 
        self.names = {0: "handle_left", 1: "handle_mid", 2: "handle_right", 3: "chassis"}

        self.state_pub = self.create_publisher(String, '/robot_shadow_states', 10)
        self.create_subscription(UInt8MultiArray, '/tcp_rx_raw', self.tcp_cb, 10)
        self.create_subscription(UInt8MultiArray, '/serial_rx_raw', self.ser_cb, 10)
        
        # 【新增】：订阅来自 Control 层的内部状态强行修改指令
        self.create_subscription(String, '/protocol_internal_cmd', self.internal_cmd_cb, 10)

    # ======================== 【新增】内部状态强制更新 ========================
    def internal_cmd_cb(self, msg):
        """
        处理 Control 层的强制写回：例如将 arrival_done 设为 False，更新 task_id
        输入格式: {"target": "chassis", "update_field": "arrival_done", "value": False, "task_id": 105}
        """
        try:
            cmd_data = json.loads(msg.data)
            target = cmd_data.get("target")
            field = cmd_data.get("update_field")
            value = cmd_data.get("value")
            task_id = cmd_data.get("task_id")

            if target == "chassis":
                if field in self.chassis_state:
                    self.chassis_state[field] = value
                if task_id is not None:
                    self.chassis_state["task_id"] = task_id
                # 状态被外部强制改变后，立刻发布一帧新影子，加速大脑层响应
                self.publish_state("chassis", self.chassis_state, 0xFF) # 0xFF 为内部虚拟指令码

            elif target in self.handle_states:
                if field in self.handle_states[target]:
                    self.handle_states[target][field] = value
                if task_id is not None:
                    self.handle_states[target]["task_id"] = task_id
                self.publish_state(target, self.handle_states[target], 0xFF)
                
        except Exception as e:
            self.get_logger().error(f"Internal CMD parse error: {e}")

    # ======================== 以下函数保持你的原样 ========================
    def parse_handle_mpu(self, name, data):
        r, p, y = struct.unpack('<fff', data[:12])
        self.handle_states[name]["mpu"] = {"r": r, "p": p, "y": y}

    def parse_handle_odom(self, name, data):
        m1, m2 = struct.unpack('<ii', data[:8])
        self.handle_states[name]["motor_odom"] = [m1, m2]

    def parse_handle_ack(self, name, data):
        pass

    def parse_chassis_status(self, data):
        vx, vy, vz, volt = struct.unpack('<hhhB', data[:7])
        self.chassis_state["speed"] = {"vx": vx, "vy": vy, "vz": vz}
        self.chassis_state["voltage"] = volt / 10.0
    
    def parse_chassis_imu_raw(self, data):
        vals = struct.unpack('<hhhhhhhhh', data[:18])
        self.chassis_state["imu_raw"]["gyro"] = list(vals[0:3])
        self.chassis_state["imu_raw"]["acc"]  = list(vals[3:6])
        self.chassis_state["imu_raw"]["mag"]  = list(vals[6:9])

    def parse_chassis_imu_euler(self, data):
        r, p, y = struct.unpack('<hhh', data[:6])
        self.chassis_state["imu_euler"] = {"roll": r / 100.0, "pitch": p / 100.0, "yaw": y / 100.0}

    def parse_chassis_odom(self, data):
        e1, e2, e3, e4 = struct.unpack('<iiii', data[:16])
        self.chassis_state["motor_encoder"] = [e1, e2, e3, e4]

    def dispatch(self, sid, cmd, data):
        device_name = self.names[sid]
        if sid < 3:
            handler = self.handle_cmd_table.get(cmd)
            if handler:
                handler(device_name, data)
                self.publish_state(device_name, self.handle_states[device_name], cmd)
        else:
            handler = self.chassis_cmd_table.get(cmd)
            if handler:
                handler(data)
                self.publish_state("chassis", self.chassis_state, cmd)

    def publish_state(self, name, state_dict, trigger_cmd):
        msg = String()
        msg.data = json.dumps({"source": name, "cmd": hex(trigger_cmd), "state": state_dict})
        self.state_pub.publish(msg)

    def tcp_cb(self, msg):
        if msg.data: self.stream_unpack(msg.data[0], msg.data[1:])

    def ser_cb(self, msg):
        if msg.data: self.stream_unpack(3, msg.data)

    def stream_unpack(self, sid, raw_bytes):
        buf = self.bufs[sid]
        buf.extend(raw_bytes)
        while len(buf) >= 5:
            if buf[0] != 0xFF or buf[1] != 0xFB:
                buf.pop(0); continue
            length = buf[2]
            total_len = length + 3
            if len(buf) < total_len: break
            packet = buf[:total_len]
            if sum(packet[2:-1]) % 256 == packet[-1]:
                self.dispatch(sid, packet[3], packet[4:-1])
                del buf[:total_len]
            else:
                buf.pop(0)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ProtocolNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
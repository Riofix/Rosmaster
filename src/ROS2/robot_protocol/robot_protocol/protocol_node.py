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
                "action_done": False    # 指示：抓手垂直升降/抓取流程完成
            } for name in ["handle_left", "handle_mid", "handle_right"]
        }

        # 2. Chassis (串口底盘) 的专属状态结构
        self.chassis_state = {
            "linear_vel": 0.0,
            "angular_vel": 0.0,
            "imu": {"acc": [0.0, 0.0, 0.0], "gyro": [0.0, 0.0, 0.0]},
            "voltage": 0.0,
            "error_code": 0,
            "arrival_done": False
        }

        # ======================== 解析表驱动隔离 ========================

        # TCP Handle 的指令解析映射
        self.handle_cmd_table = {
            0x64: self.parse_handle_mpu,   # MPU 上报
            0x65: self.parse_handle_odom,  # 里程计上报
            0x60: self.parse_handle_ack,   # 通用应答
            0x62: self.parse_handle_bldc,  # BLDC 上报
            0x66: self.parse_handle_color, # 颜色上报

            #这里需要补充代码还没写完表驱动
        }

        # Serial Chassis 的指令解析映射
        self.chassis_cmd_table = {
            0x61: self.parse_chassis_status, # 底盘状态包
            0x64: self.parse_chassis_imu,    # 底盘 IMU 包 (虽 CMD 同，但进不同函数)
            #这里需要补充代码还没写完表驱动
        }

        # ======================== 基础配置 ========================
        self.bufs = {i: bytearray() for i in range(4)} # 0-2:TCP, 3:Serial
        self.names = {0: "handle_left", 1: "handle_mid", 2: "handle_right", 3: "chassis"}

        self.state_pub = self.create_publisher(String, '/robot_shadow_states', 10)
        self.create_subscription(UInt8MultiArray, '/tcp_rx_raw', self.tcp_cb, 10)
        self.create_subscription(UInt8MultiArray, '/serial_rx_raw', self.ser_cb, 10)

    # ======================== Handle 解析函数集 ========================    #这里需要补充代码还没写完表驱动

    def parse_handle_mpu(self, name, data):
        r, p, y = struct.unpack('<fff', data[:12])
        self.handle_states[name]["mpu"] = {"r": r, "p": p, "y": y}

    def parse_handle_odom(self, name, data):
        m1, m2 = struct.unpack('<ii', data[:8])
        self.handle_states[name]["motor_odom"] = [m1, m2]

    def parse_handle_ack(self, name, data):
        # 处理参数返回等
        pass

    # ======================== Chassis 解析函数集 ========================    #这里需要补充代码还没写完表驱动

    def parse_chassis_status(self, data):
        # 假设底盘 0x61 包里有速度和电压: [vel_x(f)][vel_z(f)][volts(f)]
        vx, vz, v = struct.unpack('<fff', data[:12])
        self.chassis_state["linear_vel"] = vx
        self.chassis_state["angular_vel"] = vz
        self.chassis_state["voltage"] = v

    def parse_chassis_imu(self, data):
        # 底盘专属的 IMU 原始数据解析
        pass

    # ======================== 核心分发逻辑 (双影子隔离) ========================

    def dispatch(self, sid, cmd, data):
        device_name = self.names[sid]

        if sid < 3:
            # --- 处理 Handle 设备 ---
            handler = self.handle_cmd_table.get(cmd)
            if handler:
                handler(device_name, data) # 更新 handle_states
                self.publish_state(device_name, self.handle_states[device_name], cmd)
        else:
            # --- 处理 Chassis 设备 ---
            handler = self.chassis_cmd_table.get(cmd)
            if handler:
                handler(data) # 更新 chassis_state
                self.publish_state("chassis", self.chassis_state, cmd)

    def publish_state(self, name, state_dict, trigger_cmd):
        """统一发布 JSON"""
        msg = String()
        msg.data = json.dumps({
            "source": name,
            "cmd": hex(trigger_cmd),
            "state": state_dict
        })
        self.state_pub.publish(msg)

    # ======================== 状态机保持不变 ========================

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
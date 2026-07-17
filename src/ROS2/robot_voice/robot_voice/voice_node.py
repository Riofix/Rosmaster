import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading
import json
import struct
import time


# ==========================================================================
#  VoiceNode — 语音播报 & 语音指令识别节点
#
#  串口: /dev/broadcast (CH340 USB-TTL → SU-03T 语音模块)
#  协议: AA 55 XX YY FB (5字节定长帧)
#
#  上行 (模块→主机): 用户说出命令词后, 模块通过串口上报
#  下行 (主机→模块): 主机发送指令触发模块播报
# ==========================================================================

# ── 语音命令词协议表 ──────────────────────────────────────────────
#  语义标签    | 命令词    | 播报模式 | 接收(模块→主机)  | 发送(主机→模块)
# ────────────┼──────────┼─────────┼─────────────────┼─────────────────
#  欢迎语      | 欢迎语   | 主      | AA 55 01 00 FB  | AA 55 01 00 FB
#  休息语      | 休息语   | 主      | AA 55 02 6F FB  | AA 55 02 00 FB
#  你好地瓜    | 唤醒词   | 主      | AA 55 03 00 FB  | AA 55 03 00 FB
#  增大音量    | 增大音量 | 主      | AA 55 04 00 FB  | AA 55 04 00 FB
#  减小音量    | 减小音量 | 主      | AA 55 05 00 FB  | AA 55 05 00 FB
#  最大音量    | 最大音量 | 主      | AA 55 06 00 FB  | AA 55 06 00 FB
#  中等音量    | 中等音量 | 主      | AA 55 07 00 FB  | AA 55 07 00 FB
#  最小音量    | 最小音量 | 主      | AA 55 08 00 FB  | AA 55 08 00 FB
#  开启播报    | 开播报   | 主      | AA 55 09 00 FB  | AA 55 09 00 FB
#  关闭播报    | 关播报   | 主      | AA 55 0A 00 FB  | AA 55 0A 00 FB
#  小车停车    | 命令词   | 主      | AA 55 00 01 FB  | AA 55 00 01 FB
#  ... (其他车辆控制命令词)
#  地瓜启动    | 命令词   | 被      | AA 55 FF 01 FB  | AA 55 FF 01 FB
#  地瓜初始化  | 命令词   | 被      | AA 55 FF 02 FB  | AA 55 FF 02 FB
# ─────────────────────────────────────────────────────────────────

# 协议常量
VOICE_HEADER = bytes([0xAA, 0x55])
VOICE_FOOTER = 0xFB
VOICE_FRAME_LEN = 5

# 命令词 → 协议帧映射 (发送)
CMD_SEND_TABLE = {
    # id  →  AA 55 [byte2] [byte3] FB
    1:   bytes([0xAA, 0x55, 0x01, 0x00, 0xFB]),   # 欢迎语
    2:   bytes([0xAA, 0x55, 0x02, 0x00, 0xFB]),   # 休息语
    3:   bytes([0xAA, 0x55, 0x03, 0x00, 0xFB]),   # 你好地瓜
    4:   bytes([0xAA, 0x55, 0x04, 0x00, 0xFB]),   # 增大音量
    5:   bytes([0xAA, 0x55, 0x05, 0x00, 0xFB]),   # 减小音量
    6:   bytes([0xAA, 0x55, 0x06, 0x00, 0xFB]),   # 最大音量
    7:   bytes([0xAA, 0x55, 0x07, 0x00, 0xFB]),   # 中等音量
    8:   bytes([0xAA, 0x55, 0x08, 0x00, 0xFB]),   # 最小音量
    9:   bytes([0xAA, 0x55, 0x09, 0x00, 0xFB]),   # 开启播报
    10:  bytes([0xAA, 0x55, 0x0A, 0x00, 0xFB]),   # 关闭播报
    11:  bytes([0xAA, 0x55, 0x00, 0x01, 0xFB]),   # 小车停车
    12:  bytes([0xAA, 0x55, 0x00, 0x02, 0xFB]),   # 停车
    13:  bytes([0xAA, 0x55, 0x00, 0x03, 0xFB]),   # 小车休眠
    14:  bytes([0xAA, 0x55, 0x00, 0x04, 0xFB]),   # 小车前进
    15:  bytes([0xAA, 0x55, 0x00, 0x05, 0xFB]),   # 小车后退
    16:  bytes([0xAA, 0x55, 0x00, 0x06, 0xFB]),   # 小车左转
    17:  bytes([0xAA, 0x55, 0x00, 0x07, 0xFB]),   # 小车右转
    18:  bytes([0xAA, 0x55, 0x00, 0x08, 0xFB]),   # 小车左旋
    19:  bytes([0xAA, 0x55, 0x00, 0x09, 0xFB]),   # 小车右旋
    20:  bytes([0xAA, 0x55, 0xFF, 0x01, 0xFB]),   # 地瓜启动
    21:  bytes([0xAA, 0x55, 0xFF, 0x02, 0xFB]),   # 地瓜初始化
}

# 接收帧 → 命令词映射 (用于识别用户语音指令)
# key: (byte2, byte3) → value: (id, label, mode)
RX_FRAME_TABLE = {}
for _id, _frame in CMD_SEND_TABLE.items():
    RX_FRAME_TABLE[(_frame[2], _frame[3])] = _id


class VoiceNode(Node):
    def __init__(self):
        super().__init__('voice_node')

        # ── 串口参数 ──────────────────────────────────────────
        self.declare_parameter('port', '/dev/broadcast')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        # ── ROS 接口 ──────────────────────────────────────────
        # 订阅: 大脑或其他节点请求播报 → /voice_broadcast
        self.create_subscription(String, '/voice_broadcast', self.broadcast_cb, 10)

        # 发布: 语音模块识别到的命令词 → /voice_cmd
        self.cmd_pub = self.create_publisher(String, '/voice_cmd', 10)

        # ── 初始化串口 ────────────────────────────────────────
        self.ser = None
        self._rx_thread = None
        self._rx_buf = bytearray()
        self._running = True

        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            self.get_logger().info(f'Voice serial {port} opened ({baudrate}, 8N1)')

            # 启动接收线程
            self._rx_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._rx_thread.start()
            self.get_logger().info('Voice RX thread started')
        except Exception as e:
            self.get_logger().error(f'Failed to open {port}: {e}')
            self.get_logger().error('Voice node running without serial port')

    # =================================================================
    #  下行: 播报指令
    # =================================================================

    def broadcast_cb(self, msg):
        """
        接收 /voice_broadcast 指令, 通过串口发送 AA 55 协议帧给语音模块。
        消息格式: {"cmd_id": 21}  或  {"cmd_id": 1}
        """
        try:
            data = json.loads(msg.data)
            cmd_id = data.get('cmd_id', 0)

            if cmd_id in CMD_SEND_TABLE:
                frame = CMD_SEND_TABLE[cmd_id]
                self._send_frame(frame)
                label = data.get('label', f'id={cmd_id}')
                self.get_logger().info(f'[BC] 播报: {label}  →  {frame.hex(" ").upper()}')
            else:
                self.get_logger().warn(f'[BC] 未知播报 ID: {cmd_id}')

        except Exception as e:
            self.get_logger().error(f'[BC] 解析错误: {e}')

    def _send_frame(self, frame):
        """发送 AA 55 协议帧到语音模块"""
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(frame)
            except Exception as e:
                self.get_logger().error(f'[TX] 串口写入失败: {e}')

    # =================================================================
    #  上行: 接收语音指令
    # =================================================================

    def _read_loop(self):
        """后台线程: 持续读取串口, 解析 AA 55 协议帧"""
        while self._running and rclpy.ok():
            if self.ser and self.ser.is_open:
                try:
                    if self.ser.in_waiting > 0:
                        raw = self.ser.read(self.ser.in_waiting)
                        self._rx_buf.extend(raw)
                        self._parse_rx_buffer()
                except Exception as e:
                    self.get_logger().error(f'[RX] 串口读取异常: {e}')
                    break

    def _parse_rx_buffer(self):
        """从接收缓冲区中解析 AA 55 XX YY FB 帧"""
        while len(self._rx_buf) >= VOICE_FRAME_LEN:
            # 寻找帧头 AA 55
            if self._rx_buf[0] != 0xAA or self._rx_buf[1] != 0x55:
                self._rx_buf.pop(0)
                continue

            # 检查帧尾 FB
            if self._rx_buf[4] != VOICE_FOOTER:
                self._rx_buf.pop(0)
                continue

            # 取出一帧
            frame = bytes(self._rx_buf[:VOICE_FRAME_LEN])
            del self._rx_buf[:VOICE_FRAME_LEN]

            self._handle_rx_frame(frame)

    def _handle_rx_frame(self, frame):
        """处理接收到的 AA 55 协议帧"""
        byte2 = frame[2]
        byte3 = frame[3]
        key = (byte2, byte3)
        cmd_id = RX_FRAME_TABLE.get(key, None)

        if cmd_id is not None:
            self.get_logger().info(
                f'[RX] 语音指令: id={cmd_id}  →  {frame.hex(" ").upper()}'
            )

            # 发布到 /voice_cmd
            msg = String()
            msg.data = json.dumps({
                'cmd_id': cmd_id,
                'label': f'voice_cmd_{cmd_id}',
                'raw': frame.hex(' ').upper()
            })
            self.cmd_pub.publish(msg)
        else:
            self.get_logger().info(
                f'[RX] 未识别帧: {frame.hex(" ").upper()}'
            )

    # =================================================================
    #  生命周期
    # =================================================================

    def destroy_node(self):
        self._running = False
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

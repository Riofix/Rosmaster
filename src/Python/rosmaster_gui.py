#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Rosmaster 上位机 GUI 控制程序
=================================
基于 STM32F103C8T6 通信协议的可视化上位机。
支持 WiFi (TCP Server/Client) + 串口 (UART) 三种连接方式。
支持：位置模式、速度模式、急停、串口数据回显。

协议格式 (USART2 透传, 115200 8N1):
  发送帧: 0xFF 0xFC [len] [data...] [checksum]
  接收帧: 0xFF 0xFB [len] [data...] [checksum]
  len = 数据长度 + 2 (包含自身和checksum)
  checksum = len + sum(data_bytes)  (8-bit)

ESP8266 WiFi 连接说明:
  - STM32 通过 ESP8266 以 STA 模式连接路由器, 然后连接上位机 TCP Server
  - 或 ESP8266 以 AP 模式自建热点, 上位机连接热点后 TCP Client 连接
  - 协议包原封不动通过 TCP 透传
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import threading
import struct
import time
import datetime
import socket
import select
import csv
import json
import os
from collections import deque

# 尝试导入 matplotlib, 如果未安装则可视化不可用
try:
    import matplotlib
    matplotlib.use('TkAgg')
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    # 配置中文字体 (优先选完整CJK字体)
    import matplotlib.font_manager as fm
    _cn_fonts = [f.name for f in fm.fontManager.ttflist if any(
        k in f.name for k in ('Microsoft YaHei', 'SimHei', 'SimSun', 'WenQuanYi'))]
    # 排除 ExtB/Ext 等扩展子集字体
    _cn_fonts = [n for n in _cn_fonts if 'Ext' not in n]
    if _cn_fonts:
        matplotlib.rcParams['font.family'] = _cn_fonts[0]
    matplotlib.rcParams['axes.unicode_minus'] = False
    HAS_MPL = True
except ImportError:
    HAS_MPL = False

# 尝试导入 serial, 如果未安装则串口模式不可用
try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

# ======================== 协议常量 ========================
PACKET_HEADER1 = 0xFF
PACKET_HEADER2_RX = 0xFC   # 上位机 → 下位机
PACKET_HEADER2_TX = 0xFB   # 下位机 → 上位机
PROTOCOL_MAX_DATA_LEN = 64

# ---- 控制指令 ----
CMD_EN_CONTROL       = 0x60  # 使能控制
CMD_VEL_CONTROL      = 0x61  # 速度模式
CMD_POS_CONTROL      = 0x62  # 位置模式
CMD_STOP_NOW         = 0x63  # 急停
CMD_SYNC_MOTION      = 0x64  # 多机同步触发
CMD_ORIGIN_SET_O     = 0x65  # 设置单圈回零位置
CMD_ORIGIN_MODIFY    = 0x66  # 修改回零参数
CMD_ORIGIN_TRIGGER   = 0x67  # 触发回零
CMD_ORIGIN_INTERRUPT = 0x68  # 强制中断回零
CMD_MODIFY_CTRL_MODE = 0x69  # 修改开环/闭环控制模式
CMD_RESET_CUR_POS    = 0x6A  # 重置当前位置为0
CMD_RESET_CLOG_PRO   = 0x6B  # 解除堵转保护
CMD_POS_CM           = 0x78  # 位置模式(cm单位)
CMD_ACTION_GRAB       = 0x79  # 启动抓取
CMD_ACTION_DONE       = 0x7C  # 抓取完成通知(下位机→上位机)
CMD_ACTION_MOVE       = 0x7A  # 电机1点位移动 [pos_id, clockwise]
CMD_SERVO_CONTROL    = 0x6C  # 舵机角度控制
CMD_BLDC_CONTROL     = 0x6D  # 无刷电机转速控制
CMD_BLDC_STOP        = 0x6E  # 无刷电机急停
CMD_RGB_SENSOR       = 0x6F  # 颜色传感器开关
CMD_SHOW_OLED        = 0x70  # OLED显示数据
CMD_MPU_CALIB        = 0x71  # MPU校准
CMD_MPU_STREAM       = 0x72  # MPU自动上报开关
CMD_STEP_MOTOR_STREAM = 0x73 # 步进电机自动上报开关
CMD_PWM_STATE_STREAM = 0x74  # 无刷/舵机状态自动上报开关
CMD_RGB_SENSOR_STREAM = 0x75 # 颜色传感器自动上报开关

# ---- 查询指令 ----
CMD_QUERY_MPU_ATT   = 0x80  # 查询MPU姿态
CMD_QUERY_MPU_RAW   = 0x81  # 查询MPU原始数据
CMD_QUERY_COLOR_RAW = 0x82  # 查询颜色原始数据
CMD_QUERY_COLOR_RES = 0x83  # 查询颜色识别结果
CMD_QUERY_SERVO_STAT = 0x84 # 查询舵机状态
CMD_QUERY_BLDC_STAT = 0x85  # 查询无刷电机状态
CMD_QUERY_STEP_STAT = 0x86  # 查询步进电机状态
CMD_QUERY_STEP_PARAM = 0x87 # 查询步进电机参数

# ---- TX 应答 / 自动上报 ----
CMD_TX_ACK_OK       = 0x90  # 通用成功应答
CMD_TX_STREAM_MPU   = 0x5A  # MPU自动上报
CMD_TX_STREAM_STEP  = 0x5B  # 步进电机自动上报
CMD_TX_STREAM_STATE = 0x5C  # 无刷/舵机状态自动上报
CMD_TX_STREAM_COLOR = 0x5D  # 颜色传感器自动上报

# 命令名称映射 (用于日志显示)
CMD_NAMES = {
    0x60: "使能控制", 0x61: "速度模式", 0x62: "位置模式",
    0x63: "急停", 0x64: "同步触发", 0x65: "设置回零",
    0x66: "修改回零参数", 0x67: "触发回零", 0x68: "中断回零",
    0x69: "修改控制模式", 0x6A: "重置当前位置", 0x6B: "解除堵转保护",
    0x6C: "舵机控制", 0x6D: "无刷电机控制", 0x6E: "无刷电机急停",
    0x6F: "颜色传感器", 0x70: "OLED显示", 0x71: "MPU校准",
    0x78: "位置模式(cm)",
    0x72: "MPU上报开关", 0x73: "步进电机上报开关", 0x74: "PWM状态上报开关",
    0x75: "颜色传感器上报开关",
    0x80: "查询MPU姿态", 0x81: "查询MPU原始", 0x82: "查询颜色原始",
    0x83: "查询颜色结果", 0x84: "查询舵机状态", 0x85: "查询无刷状态",
    0x86: "查询步进状态", 0x87: "查询步进参数",
    0x90: "✅ ACK成功",
    0x5A: "📡 MPU上报", 0x5B: "📡 步进电机上报",
    0x5C: "📡 PWM状态上报", 0x5D: "📡 颜色上报",
}


# ======================== 协议工具函数 ========================
def calculate_checksum(packet_len: int, data: bytes) -> int:
    """计算校验和: len + sum(data_bytes), 取低8位"""
    return (packet_len + sum(data)) & 0xFF


def build_packet(data: bytes) -> bytes:
    """构建完整发送帧"""
    if len(data) > PROTOCOL_MAX_DATA_LEN:
        raise ValueError(f"数据长度超过 {PROTOCOL_MAX_DATA_LEN} 字节")
    packet_len = len(data) + 2
    cksum = calculate_checksum(packet_len, data)
    return bytes([PACKET_HEADER1, PACKET_HEADER2_RX, packet_len]) + data + bytes([cksum])


def format_hex(data: bytes) -> str:
    """将字节数据格式化为十六进制字符串"""
    return ' '.join(f'{b:02X}' for b in data)


def timestamp() -> str:
    """获取当前时间戳字符串"""
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]


# ======================== 数据模型 ========================
class MotorState:
    """步进电机状态 (对应 MotorState_t, 23字节, 小端序)"""
    __slots__ = ('addr', 'voltage_mv', 'phase_current', 'encoder_val',
                 'target_pos', 'velocity', 'current_pos', 'pos_error', 'org', 'flag')
    FMT = '<B H h H i h i i B B'

    def __init__(self):
        self.addr = 0
        self.voltage_mv = 0
        self.phase_current = 0
        self.encoder_val = 0
        self.target_pos = 0
        self.velocity = 0
        self.current_pos = 0
        self.pos_error = 0
        self.org = 0
        self.flag = 0

    @classmethod
    def unpack(cls, data: bytes):
        if len(data) < 23:
            return None
        try:
            vals = struct.unpack(cls.FMT, data[:23])
        except struct.error:
            return None
        m = cls()
        (m.addr, m.voltage_mv, m.phase_current, m.encoder_val,
         m.target_pos, m.velocity, m.current_pos, m.pos_error,
         m.org, m.flag) = vals
        return m

    @property
    def is_enabled(self) -> bool:
        return bool(self.flag & 0x01)

    @property
    def is_in_position(self) -> bool:
        return bool(self.flag & 0x02)

    @property
    def is_stalled(self) -> bool:
        return bool(self.flag & 0x04)

    @property
    def stall_protection(self) -> bool:
        return bool(self.flag & 0x08)


# ======================== 数据记录器 ========================
class DataRecorder:
    """环形缓冲区 + CSV 记录"""
    MAX_POINTS = 3000  # 30秒 @ 100Hz

    def __init__(self):
        self.timestamps: deque = deque(maxlen=self.MAX_POINTS)
        self.rolls: deque = deque(maxlen=self.MAX_POINTS)
        self.pitches: deque = deque(maxlen=self.MAX_POINTS)
        self.yaws: deque = deque(maxlen=self.MAX_POINTS)
        self.z_accels: deque = deque(maxlen=self.MAX_POINTS)
        self.m1_positions: deque = deque(maxlen=self.MAX_POINTS)
        self.m1_velocities: deque = deque(maxlen=self.MAX_POINTS)
        self.m2_positions: deque = deque(maxlen=self.MAX_POINTS)
        self.m2_velocities: deque = deque(maxlen=self.MAX_POINTS)
        self.m1_voltages: deque = deque(maxlen=self.MAX_POINTS)
        self.m1_currents: deque = deque(maxlen=self.MAX_POINTS)
        self.m2_voltages: deque = deque(maxlen=self.MAX_POINTS)
        self.m2_currents: deque = deque(maxlen=self.MAX_POINTS)
        self.servo1_angles: deque = deque(maxlen=self.MAX_POINTS)
        self.servo2_angles: deque = deque(maxlen=self.MAX_POINTS)
        self.bldc_duties: deque = deque(maxlen=self.MAX_POINTS)
        self._csv_file = None
        self._csv_writer = None
        self._csv_path = ""
        self._recording = False
        self._record_count = 0
        self._start_time = 0.0
        # 最近一次电机状态
        self.latest_m1: MotorState | None = None
        self.latest_m2: MotorState | None = None
        self.latest_mpu: tuple = (0.0, 0.0, 0.0)
        self.latest_servo: tuple = (0, 0)
        self.latest_bldc: int = 0

    @property
    def is_recording(self) -> bool:
        return self._recording

    def start_recording(self):
        if self._recording:
            return
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'log')
        os.makedirs(log_dir, exist_ok=True)
        self._csv_path = os.path.join(
            log_dir,
            f"rosmaster_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )
        self._csv_file = open(self._csv_path, 'w', newline='', encoding='utf-8')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            'timestamp_ms', 'type',
            'roll', 'pitch', 'yaw', 'z_accel',
            'm1_pos', 'm1_vel', 'm1_voltage', 'm1_current',
            'm2_pos', 'm2_vel', 'm2_voltage', 'm2_current',
            'servo1', 'servo2', 'bldc_duty'
        ])
        self._recording = True
        self._record_count = 0
        self._start_time = time.time()

    def stop_recording(self) -> str:
        if not self._recording:
            return ""
        self._recording = False
        if self._csv_file:
            self._csv_file.close()
            self._csv_file = None
        return self._csv_path

    def add_mpu(self, ts: float, roll: float, pitch: float, yaw: float, z_accel: float = 0.0):
        self.timestamps.append(ts)
        self.rolls.append(roll)
        self.pitches.append(pitch)
        self.yaws.append(yaw)
        self.z_accels.append(z_accel)
        self.latest_mpu = (roll, pitch, yaw)
        if self._recording:
            self._csv_writer.writerow([
                f"{ts:.3f}", 'mpu', roll, pitch, yaw, z_accel,
                '', '', '', '', '', '', '', '', '', '', ''
            ])
            self._record_count += 1

    def add_motor(self, ts: float, ms: MotorState):
        if ms.addr == 1:
            self.latest_m1 = ms
            self.m1_positions.append(ms.current_pos)
            self.m1_velocities.append(ms.velocity)
            self.m1_voltages.append(ms.voltage_mv)
            self.m1_currents.append(ms.phase_current)
            # 同步时间戳
            if len(self.m1_positions) > len(self.timestamps):
                self.timestamps.append(ts)
            if self._recording:
                self._csv_writer.writerow([
                    f"{ts:.3f}", 'motor', '', '', '', '',
                    ms.current_pos, ms.velocity, ms.voltage_mv, ms.phase_current,
                    '', '', '', '', '', '', ''
                ])
                self._record_count += 1
        elif ms.addr == 2:
            self.latest_m2 = ms
            self.m2_positions.append(ms.current_pos)
            self.m2_velocities.append(ms.velocity)
            self.m2_voltages.append(ms.voltage_mv)
            self.m2_currents.append(ms.phase_current)
            if self._recording:
                self._csv_writer.writerow([
                    f"{ts:.3f}", 'motor', '', '', '', '',
                    '', '', '', '',
                    ms.current_pos, ms.velocity, ms.voltage_mv, ms.phase_current,
                    '', '', ''
                ])
                self._record_count += 1

    def add_pwm(self, ts: float, servo1: int, servo2: int, bldc: int):
        self.servo1_angles.append(servo1)
        self.servo2_angles.append(servo2)
        self.bldc_duties.append(bldc)
        self.latest_servo = (servo1, servo2)
        self.latest_bldc = bldc
        if self._recording:
            self._csv_writer.writerow([
                f"{ts:.3f}", 'pwm', '', '', '', '',
                '', '', '', '', '', '', '', '',
                servo1, servo2, bldc
            ])
            self._record_count += 1

    def clear(self):
        for attr in ('timestamps', 'rolls', 'pitches', 'yaws', 'z_accels',
                     'm1_positions', 'm1_velocities', 'm2_positions', 'm2_velocities',
                     'm1_voltages', 'm1_currents', 'm2_voltages', 'm2_currents',
                     'servo1_angles', 'servo2_angles', 'bldc_duties'):
            getattr(self, attr).clear()
        self.latest_m1 = None
        self.latest_m2 = None
        self.latest_mpu = (0.0, 0.0, 0.0)
        self.latest_servo = (0, 0)
        self.latest_bldc = 0
        self._record_count = 0


# ======================== 协议解析器 (纯数据, 与传输层无关) ========================
class ProtocolParser:
    """协议帧解析状态机 (线程安全)"""

    def __init__(self, on_packet):
        self._on_packet = on_packet
        self._buffer = bytearray()
        self._lock = threading.Lock()

    def feed(self, raw: bytes):
        """喂入原始字节"""
        with self._lock:
            self._buffer.extend(raw)
        self._parse()

    def reset(self):
        with self._lock:
            self._buffer.clear()

    def _parse(self):
        """从缓冲区解析完整数据帧"""
        while True:
            with self._lock:
                buf = bytes(self._buffer)

            idx = buf.find(PACKET_HEADER1)
            if idx < 0:
                break
            if idx > 0:
                with self._lock:
                    del self._buffer[:idx]
                continue

            if len(buf) < 4:
                break

            header2 = buf[1]
            if header2 != PACKET_HEADER2_TX:
                with self._lock:
                    if len(self._buffer) > 1:
                        del self._buffer[1]
                    else:
                        del self._buffer[0]
                continue

            packet_len = buf[2]
            if packet_len < 2 or packet_len > (PROTOCOL_MAX_DATA_LEN + 2):
                with self._lock:
                    if len(self._buffer) > 1:
                        del self._buffer[1]
                    else:
                        del self._buffer[0]
                continue

            total_len = 3 + (packet_len - 2) + 1
            if len(buf) < total_len:
                break

            frame = buf[:total_len]
            with self._lock:
                del self._buffer[:total_len]

            data_bytes = frame[3:3 + packet_len - 2]
            if frame[-1] != calculate_checksum(packet_len, data_bytes):
                continue

            self._on_packet(frame, data_bytes)


# ======================== 通信基类 ========================
class CommBase:
    """通信抽象基类, 统一 Serial / TCP Server / TCP Client 接口"""

    def __init__(self, on_packet, on_status, on_raw):
        self._parser = ProtocolParser(on_packet)
        self._on_status = on_status
        self._on_raw = on_raw
        self._running = False
        self._thread: threading.Thread | None = None

    @property
    def is_connected(self) -> bool:
        raise NotImplementedError

    def connect(self, **kwargs) -> bool:
        raise NotImplementedError

    def disconnect(self):
        raise NotImplementedError

    def send(self, data: bytes) -> bool:
        raise NotImplementedError

    def send_packet(self, data: bytes) -> bool:
        """构建协议包并发送"""
        packet = build_packet(data)
        return self.send(packet)

    def _notify_status(self, connected: bool, msg: str):
        self._on_status(connected, msg)

    def _notify_raw(self, raw: bytes):
        self._on_raw(raw)

    def _feed_parser(self, raw: bytes):
        self._parser.feed(raw)


# ======================== TCP Server 通信 (ESP8266 作为 Client 连接) ========================


class TcpServerComm(CommBase):
    """TCP Server — 多 ESP8266 同时连接"""
    MAX_DEVICES = 3

    def __init__(self, on_packet, on_status, on_raw, on_dev_conn, on_dev_disc):
        super().__init__(on_packet, on_status, on_raw)
        self._server_socket: socket.socket | None = None
        self._port: int = 0
        self._on_dev_conn = on_dev_conn
        self._on_dev_disc = on_dev_disc
        self._dev_socks: list = [None] * self.MAX_DEVICES
        self._dev_addrs: list = [""] * self.MAX_DEVICES
        self._dev_parsers: list = [ProtocolParser(lambda f,d,i=i: on_packet(f,d,i)) for i in range(self.MAX_DEVICES)]

    @property
    def is_connected(self) -> bool:
        return any(s is not None for s in self._dev_socks)

    def device_connected(self, idx: int) -> bool:
        return self._dev_socks[idx] is not None

    def connect(self, port: int = 3456, **kwargs) -> bool:
        try:
            self._port = port
            self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._server_socket.settimeout(1.0)
            self._server_socket.bind(('0.0.0.0', port))
            self._server_socket.listen(self.MAX_DEVICES)
            self._running = True
            self._thread = threading.Thread(target=self._server_loop, daemon=True)
            self._thread.start()
            self._notify_status(False, f"TCP Server 启动, 端口 {port}")
            return True
        except OSError as e:
            self._notify_status(False, f"TCP Server 失败: {e}")
            return False

    def disconnect(self):
        self._running = False
        for i in range(self.MAX_DEVICES):
            self._close_dev(i)
        if self._server_socket:
            try: self._server_socket.close()
            except OSError: pass
            self._server_socket = None
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._notify_status(False, "TCP Server 已停止")

    def _close_dev(self, idx: int):
        if self._dev_socks[idx]:
            try: self._dev_socks[idx].close()
            except OSError: pass
            self._dev_socks[idx] = None
            self._dev_addrs[idx] = ""
        self._dev_parsers[idx].reset()

    def send(self, data: bytes, device_idx: int = 0) -> bool:
        sock = self._dev_socks[device_idx]
        if sock and self._running:
            try: sock.sendall(data); return True
            except OSError:
                self._close_dev(device_idx)
                self._on_dev_disc(device_idx)
                return False
        return False

    def send_packet(self, data: bytes, device_idx: int = 0) -> bool:
        return self.send(build_packet(data), device_idx)

    def _find_slot(self) -> int:
        for i in range(self.MAX_DEVICES):
            if self._dev_socks[i] is None: return i
        return -1

    def _server_loop(self):
        watch = [self._server_socket]
        while self._running:
            try: readable, _, _ = select.select(watch, [], [], 0.5)
            except (OSError, ValueError): break
            for s in readable:
                if s is self._server_socket:
                    try:
                        client, addr = self._server_socket.accept()
                        idx = self._find_slot()
                        if idx < 0: client.close(); continue
                        client.settimeout(0.1)
                        self._dev_socks[idx] = client
                        self._dev_addrs[idx] = f"{addr[0]}:{addr[1]}"
                        watch.append(client)
                        self._on_dev_conn(idx, self._dev_addrs[idx])
                    except OSError: pass
                else:
                    idx = next((i for i in range(self.MAX_DEVICES) if self._dev_socks[i] is s), -1)
                    if idx < 0: continue
                    try:
                        data = s.recv(4096)
                        if data:
                            self._notify_raw(data)
                            self._dev_parsers[idx].feed(data)
                        else:
                            watch.remove(s); self._close_dev(idx); self._on_dev_disc(idx)
                    except socket.timeout: continue
                    except (OSError, ConnectionResetError, ConnectionAbortedError):
                        if s in watch: watch.remove(s)
                        self._close_dev(idx); self._on_dev_disc(idx)


class TcpClientComm(CommBase):
    """TCP Client — PC 主动连接"""
    def __init__(self, on_packet, on_status, on_raw):
        super().__init__(on_packet, on_status, on_raw)
        self._sock: socket.socket | None = None

    @property
    def is_connected(self) -> bool:
        return self._sock is not None and self._running

    def device_connected(self, idx: int = 0) -> bool:
        return self.is_connected

    def connect(self, host: str = "192.168.4.1", port: int = 8080, **kwargs) -> bool:
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.settimeout(3.0); self._sock.connect((host, port))
            self._sock.settimeout(0.1)
            self._running = True
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
            self._notify_status(True, f"已连接 {host}:{port}")
            return True
        except OSError as e:
            self._notify_status(False, f"TCP 连接失败: {e}")
            if self._sock: self._sock.close(); self._sock = None
            return False

    def disconnect(self):
        self._running = False
        if self._sock:
            try: self._sock.shutdown(socket.SHUT_RDWR)
            except OSError: pass
            self._sock.close(); self._sock = None
        if self._thread and self._thread.is_alive(): self._thread.join(timeout=2.0)
        self._parser.reset()
        self._notify_status(False, "TCP 已断开")

    def send(self, data: bytes, device_idx: int = 0) -> bool:
        if self._sock and self._running:
            try: self._sock.sendall(data); return True
            except OSError: return False
        return False

    def send_packet(self, data: bytes, device_idx: int = 0) -> bool:
        return self.send(build_packet(data))

    def _read_loop(self):
        while self._running and self._sock:
            try:
                data = self._sock.recv(4096)
                if data: self._notify_raw(data); self._feed_parser(data)
                else: self._notify_status(False, "TCP 关闭"); self._running = False; break
            except socket.timeout: continue
            except OSError:
                if self._running: self._notify_status(False, "TCP 断开"); self._running = False
                break


class SerialComm(CommBase):
    """串口通信"""
    def __init__(self, on_packet, on_status, on_raw):
        super().__init__(on_packet, on_status, on_raw)
        self._ser = None

    @property
    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def device_connected(self, idx: int = 0) -> bool:
        return self.is_connected

    def connect(self, port: str = "", baudrate: int = 115200, **kwargs) -> bool:
        if not HAS_SERIAL:
            self._notify_status(False, "pyserial 未安装"); return False
        try:
            self._ser = serial.Serial(port=port, baudrate=baudrate,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=0.05)
            self._running = True
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
            self._notify_status(True, f"串口 {port} @ {baudrate}")
            return True
        except serial.SerialException as e:
            self._notify_status(False, f"串口失败: {e}"); return False

    def disconnect(self):
        self._running = False
        if self._thread and self._thread.is_alive(): self._thread.join(timeout=1.0)
        if self._ser and self._ser.is_open: self._ser.close()
        self._parser.reset()
        self._notify_status(False, "串口断开")

    def send(self, data: bytes, device_idx: int = 0) -> bool:
        if self._ser and self._ser.is_open:
            try: self._ser.write(data); return True
            except serial.SerialException: return False
        return False

    def send_packet(self, data: bytes, device_idx: int = 0) -> bool:
        return self.send(build_packet(data))

    def _read_loop(self):
        while self._running and self._ser and self._ser.is_open:
            try:
                if self._ser.in_waiting > 0:
                    raw = self._ser.read(self._ser.in_waiting)
                    if raw: self._notify_raw(raw); self._feed_parser(raw)
                else: time.sleep(0.01)
            except serial.SerialException: break
            except Exception: time.sleep(0.05)


# ======================== GUI 主界面 ========================
class RosmasterGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Rosmaster 多设备控制台")
        self.root.geometry("1400x880")
        self.root.minsize(1000, 650)

        _style = ttk.Style()
        _avail = set(tk.font.families())
        for _fn in ('Microsoft YaHei', 'SimHei', 'SimSun', 'FangSong', 'KaiTi'):
            if _fn in _avail:
                _style.theme_use('default')
                _style.configure('.', font=(_fn, 9))
                _style.configure('TLabelframe.Label', font=(_fn, 9))
                break

        self.comm: CommBase | None = None

        # 设备管理
        self._device_count = 3
        self._current_device = 0
        self._device_names = ["抓手1", "抓手2", "抓手3"]
        self._device_online = [False, False, False]
        self._config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'devices.json')
        self._load_device_config()
        self.recorders = [DataRecorder() for _ in range(self._device_count)]

        self.latest_mpu: tuple = (0, 0, 0)
        self.latest_servo_angles: tuple = (0, 0)
        self.latest_bldc_duty: int = 0
        self._plot_job = None

        # GUI 变量
        self.status_var = tk.StringVar(value="未连接")
        self.auto_scroll_var = tk.BooleanVar(value=True)
        self.show_hex_var = tk.BooleanVar(value=True)
        self.serial_port_var = tk.StringVar()
        self.serial_baud_var = tk.StringVar(value="115200")
        self.tcp_svr_port_var = tk.StringVar(value="3456")
        self.tcp_cli_host_var = tk.StringVar(value="192.168.4.1")
        self.tcp_cli_port_var = tk.StringVar(value="8080")
        self.query_addr_var = tk.IntVar(value=1)
        self.step_stream_var = tk.BooleanVar(value=False)
        self.mpu_stream_var = tk.BooleanVar(value=False)
        self.pwm_stream_var = tk.BooleanVar(value=False)

        # 共享电机控制变量
        self._cur_motor = tk.IntVar(value=1)
        self._cur_en = tk.BooleanVar(value=True)
        self._cur_en_sync = tk.BooleanVar(value=False)
        self._cur_vel_dir = tk.BooleanVar(value=False)
        self._cur_vel_speed = tk.IntVar(value=500)
        self._cur_vel_acc = tk.IntVar(value=100)
        self._cur_vel_sync = tk.BooleanVar(value=False)
        self._cur_pos_dir = tk.BooleanVar(value=False)
        self._cur_pos_speed = tk.IntVar(value=500)
        self._cur_pos_acc = tk.IntVar(value=100)
        self._cur_pos_dist = tk.DoubleVar(value=10.0)
        self._cur_pos_abs = tk.BooleanVar(value=True)
        self._cur_pos_sync = tk.BooleanVar(value=False)

        self.log_lines: deque = deque(maxlen=1000)
        self._build_ui()
        self._refresh_serial_ports()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    @property
    def recorder(self):
        return self.recorders[self._current_device]

    # ========== 设备管理 ==========
    def _load_device_config(self):
        try:
            with open(self._config_path, 'r', encoding='utf-8') as f:
                cfg = json.load(f)
            for i, d in enumerate(cfg.get('devices', [])[:3]):
                if d.get('name'): self._device_names[i] = str(d['name'])
        except (FileNotFoundError, json.JSONDecodeError): pass

    def _save_device_config(self):
        try:
            with open(self._config_path, 'w', encoding='utf-8') as f:
                json.dump({'devices': [{'name': self._device_names[i]} for i in range(3)]}, f, ensure_ascii=False, indent=2)
        except OSError: pass

    def _switch_device(self, idx: int):
        if idx == self._current_device: return
        self._current_device = idx
        for i, btn in enumerate(self._device_btns):
            btn.configure(text=f"{'▶' if i==idx else '  '}设备{i+1}")
        for i, lbl in enumerate(self._device_labels):
            status = "●在线" if self._device_online[i] else "○离线"
            fg = "#2E7D32" if self._device_online[i] else "gray"
            lbl.config(text=f"{self._device_names[i]} {status}", fg=fg)

    def _on_dev_connect(self, idx: int, addr: str):
        self._device_online[idx] = True
        self.root.after(0, lambda: self._update_dev_display(idx))
        self._log("info", f"设备{idx+1}({self._device_names[idx]}) 已连接 {addr}")

    def _on_dev_disconnect(self, idx: int):
        self._device_online[idx] = False
        self.root.after(0, lambda: self._update_dev_display(idx))
        self._log("info", f"设备{idx+1}({self._device_names[idx]}) 已断开")

    def _update_dev_display(self, idx: int):
        fg = "#2E7D32" if self._device_online[idx] else "gray"
        s = "●在线" if self._device_online[idx] else "○离线"
        self._device_labels[idx].config(text=f"{self._device_names[idx]} {s}", fg=fg)
        is_cur = (idx == self._current_device)
        for i, btn in enumerate(self._device_btns):
            btn.configure(text=f"{'▶' if (i==self._current_device) else '  '}设备{i+1}")

    def _rename_device(self, idx: int):
        from tkinter import simpledialog
        new_name = simpledialog.askstring("重命名", f"设备{idx+1} 名称:", initialvalue=self._device_names[idx])
        if new_name:
            self._device_names[idx] = new_name
            self._save_device_config()
            self._update_dev_display(idx)

    # ========== UI 构建 ==========
    def _build_ui(self):
        top_bar = ttk.Frame(self.root, padding=5)
        top_bar.pack(fill=tk.X, side=tk.TOP)

        ttk.Label(top_bar, text="连接方式:").pack(side=tk.LEFT, padx=(0,5))
        self.mode_combo = ttk.Combobox(top_bar,
            values=["TCP Server (ESP8266)", "TCP Client", "串口 (UART)"], width=24, state="readonly")
        self.mode_combo.current(0)
        self.mode_combo.bind("<<ComboboxSelected>>", self._on_mode_change)
        self.mode_combo.pack(side=tk.LEFT, padx=2)

        self.mode_params_frame = ttk.Frame(top_bar)
        self.mode_params_frame.pack(side=tk.LEFT, padx=10)
        self._build_mode_params_ui()

        self.connect_btn = ttk.Button(top_bar, text="启动监听", width=10, command=self._toggle_connect)
        self.connect_btn.pack(side=tk.LEFT, padx=(5,5))

        # 设备选择器
        ttk.Separator(top_bar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8)
        ttk.Label(top_bar, text="设备:").pack(side=tk.LEFT)
        self._device_btns = []
        self._device_labels = []
        for i in range(3):
            btn = ttk.Button(top_bar, text=f"设备{i+1}", width=6,
                             command=lambda x=i: self._switch_device(x))
            btn.pack(side=tk.LEFT, padx=2)
            self._device_btns.append(btn)
            lbl = tk.Label(top_bar, text=f"{self._device_names[i]} ○离线",
                          font=("",9), fg="gray", cursor="hand2")
            lbl.pack(side=tk.LEFT, padx=(0,8))
            lbl.bind("<Double-Button-1>", lambda e, x=i: self._rename_device(x))
            self._device_labels.append(lbl)

        self.status_label = ttk.Label(top_bar, textvariable=self.status_var, foreground="gray")
        self.status_label.pack(side=tk.LEFT, padx=10)

        notebook = ttk.Notebook(self.root, padding=3)
        notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=2)

        motor_frame = ttk.Frame(notebook)
        notebook.add(motor_frame, text="  电机控制  ")
        sensor_frame = ttk.Frame(notebook)
        notebook.add(sensor_frame, text="  传感器 & 数据  ")
        monitor_frame = ttk.Frame(notebook)
        notebook.add(monitor_frame, text="  通信监视器  ")

        self._build_motor_tab(motor_frame)
        self._build_sensor_tab(sensor_frame)
        self._build_monitor_tab(monitor_frame)

        bottom_bar = ttk.Frame(self.root, padding=5)
        bottom_bar.pack(fill=tk.X, side=tk.BOTTOM)
        self.estop_btn = tk.Button(bottom_bar, text="⚠ 紧 急 停 止 ⚠",
            font=("Microsoft YaHei", 14, "bold"), fg="white", bg="#D32F2F",
            activebackground="#B71C1C", activeforeground="white",
            relief=tk.RAISED, borderwidth=3, cursor="hand2", height=2,
            command=self._emergency_stop)
        self.estop_btn.pack(fill=tk.X, padx=20, pady=5)

    def _build_mode_params_ui(self):
        for w in self.mode_params_frame.winfo_children(): w.destroy()
        mode_idx = self.mode_combo.current()
        if mode_idx == 0:
            ttk.Label(self.mode_params_frame, text="监听端口:").pack(side=tk.LEFT)
            ttk.Entry(self.mode_params_frame, textvariable=self.tcp_svr_port_var, width=7).pack(side=tk.LEFT, padx=3)
            ttk.Label(self.mode_params_frame, text="(ESP8266 自动连接)", foreground="gray").pack(side=tk.LEFT, padx=3)
        elif mode_idx == 1:
            ttk.Label(self.mode_params_frame, text="IP:").pack(side=tk.LEFT)
            ttk.Entry(self.mode_params_frame, textvariable=self.tcp_cli_host_var, width=14).pack(side=tk.LEFT, padx=3)
            ttk.Label(self.mode_params_frame, text="端口:").pack(side=tk.LEFT)
            ttk.Entry(self.mode_params_frame, textvariable=self.tcp_cli_port_var, width=7).pack(side=tk.LEFT, padx=3)
        elif mode_idx == 2:
            ttk.Label(self.mode_params_frame, text="串口:").pack(side=tk.LEFT)
            self.serial_port_combo = ttk.Combobox(self.mode_params_frame,
                textvariable=self.serial_port_var, width=12, state="readonly")
            self.serial_port_combo.pack(side=tk.LEFT, padx=2)
            ttk.Button(self.mode_params_frame, text="🔄", width=3, command=self._refresh_serial_ports).pack(side=tk.LEFT)
            ttk.Label(self.mode_params_frame, text=" 波特率:").pack(side=tk.LEFT, padx=(8,0))
            ttk.Combobox(self.mode_params_frame, textvariable=self.serial_baud_var,
                values=["9600","19200","38400","57600","115200","230400"], width=8, state="readonly").pack(side=tk.LEFT, padx=2)
            self._refresh_serial_ports()

    def _on_mode_change(self, event=None):
        if self.comm and self.comm.is_connected:
            messagebox.showwarning("注意", "请先断开当前连接再切换模式"); return
        self._build_mode_params_ui()
        self._update_connect_btn_text()

    def _update_connect_btn_text(self):
        texts = {0: "启动监听", 1: "连接", 2: "连接"}
        self.connect_btn.config(text=texts.get(self.mode_combo.current(), "连接"))

    def _get_connect_btn_text(self) -> str:
        return "启动监听" if self.mode_combo.current() == 0 else "连接"

    # ========== 电机控制页 ==========
    def _build_motor_tab(self, parent: ttk.Frame):
        top_row = ttk.Frame(parent); top_row.pack(fill=tk.X, padx=5, pady=5)
        ttk.Label(top_row, text="当前电机:").pack(side=tk.LEFT)
        ttk.Radiobutton(top_row, text="电机1", variable=self._cur_motor, value=1,
                        command=lambda: self._on_motor_sel()).pack(side=tk.LEFT, padx=3)
        ttk.Radiobutton(top_row, text="电机2", variable=self._cur_motor, value=2,
                        command=lambda: self._on_motor_sel()).pack(side=tk.LEFT, padx=3)
        ttk.Checkbutton(top_row, text="使能", variable=self._cur_en).pack(side=tk.LEFT, padx=10)
        ttk.Checkbutton(top_row, text="同步", variable=self._cur_en_sync).pack(side=tk.LEFT, padx=3)
        ttk.Button(top_row, text="发送", width=5,
                   command=lambda: self._cmd_enable(self._cur_motor.get(), self._cur_en.get(), self._cur_en_sync.get())).pack(side=tk.LEFT, padx=5)

        ctrl_p = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        ctrl_p.pack(fill=tk.X, padx=5, pady=5)

        # 速度模式
        vf = ttk.LabelFrame(ctrl_p, text="速度模式 (0x61)", padding=5)
        ctrl_p.add(vf, weight=1)
        r1 = ttk.Frame(vf); r1.pack(fill=tk.X, pady=2)
        ttk.Label(r1, text="方向:", width=5).pack(side=tk.LEFT)
        self._vel_dir_l1 = ttk.Radiobutton(r1, text="正转(CW)", variable=self._cur_vel_dir, value=False)
        self._vel_dir_l1.pack(side=tk.LEFT, padx=2)
        self._vel_dir_l2 = ttk.Radiobutton(r1, text="反转(CCW)", variable=self._cur_vel_dir, value=True)
        self._vel_dir_l2.pack(side=tk.LEFT, padx=2)
        r2 = ttk.Frame(vf); r2.pack(fill=tk.X, pady=2)
        ttk.Label(r2, text="速度:", width=5).pack(side=tk.LEFT)
        ttk.Scale(r2, from_=0, to=3000, variable=self._cur_vel_speed, orient=tk.HORIZONTAL).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        ttk.Entry(r2, textvariable=self._cur_vel_speed, width=6).pack(side=tk.RIGHT)
        r3 = ttk.Frame(vf); r3.pack(fill=tk.X, pady=2)
        ttk.Label(r3, text="加速度:", width=5).pack(side=tk.LEFT)
        ttk.Scale(r3, from_=0, to=255, variable=self._cur_vel_acc, orient=tk.HORIZONTAL).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        ttk.Entry(r3, textvariable=self._cur_vel_acc, width=6).pack(side=tk.RIGHT)
        r4 = ttk.Frame(vf); r4.pack(fill=tk.X, pady=2)
        ttk.Checkbutton(r4, text="同步", variable=self._cur_vel_sync).pack(side=tk.LEFT)
        ttk.Button(r4, text="▶ 启动", width=8,
                   command=lambda: self._cmd_velocity(self._cur_motor.get(), self._cur_vel_dir.get(),
                       self._cur_vel_speed.get(), self._cur_vel_acc.get(), self._cur_vel_sync.get())).pack(side=tk.RIGHT, padx=5)

        # 位置模式 cm
        pf = ttk.LabelFrame(ctrl_p, text="位置模式 (0x78, cm)", padding=5)
        ctrl_p.add(pf, weight=1)
        p1 = ttk.Frame(pf); p1.pack(fill=tk.X, pady=2)
        ttk.Label(p1, text="方向:", width=5).pack(side=tk.LEFT)
        self._pos_dir_l1 = ttk.Radiobutton(p1, text="正转(CW)", variable=self._cur_pos_dir, value=False)
        self._pos_dir_l1.pack(side=tk.LEFT, padx=2)
        self._pos_dir_l2 = ttk.Radiobutton(p1, text="反转(CCW)", variable=self._cur_pos_dir, value=True)
        self._pos_dir_l2.pack(side=tk.LEFT, padx=2)
        p2 = ttk.Frame(pf); p2.pack(fill=tk.X, pady=2)
        ttk.Label(p2, text="速度:", width=5).pack(side=tk.LEFT)
        ttk.Scale(p2, from_=0, to=3000, variable=self._cur_pos_speed, orient=tk.HORIZONTAL).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        ttk.Entry(p2, textvariable=self._cur_pos_speed, width=6).pack(side=tk.RIGHT)
        p3 = ttk.Frame(pf); p3.pack(fill=tk.X, pady=2)
        ttk.Label(p3, text="加速度:", width=5).pack(side=tk.LEFT)
        ttk.Scale(p3, from_=0, to=255, variable=self._cur_pos_acc, orient=tk.HORIZONTAL).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        ttk.Entry(p3, textvariable=self._cur_pos_acc, width=6).pack(side=tk.RIGHT)
        p4 = ttk.Frame(pf); p4.pack(fill=tk.X, pady=2)
        ttk.Label(p4, text="距离:", width=5).pack(side=tk.LEFT)
        ttk.Entry(p4, textvariable=self._cur_pos_dist, width=8).pack(side=tk.LEFT, padx=5)
        ttk.Label(p4, text="cm").pack(side=tk.LEFT)
        p5 = ttk.Frame(pf); p5.pack(fill=tk.X, pady=2)
        ttk.Label(p5, text="模式:", width=5).pack(side=tk.LEFT)
        ttk.Radiobutton(p5, text="绝对", variable=self._cur_pos_abs, value=True).pack(side=tk.LEFT, padx=2)
        ttk.Radiobutton(p5, text="相对", variable=self._cur_pos_abs, value=False).pack(side=tk.LEFT, padx=2)
        p6 = ttk.Frame(pf); p6.pack(fill=tk.X, pady=2)
        ttk.Checkbutton(p6, text="同步", variable=self._cur_pos_sync).pack(side=tk.LEFT)
        ttk.Button(p6, text="▶ 启动", width=8,
                   command=lambda: self._pos_cmd()).pack(side=tk.RIGHT, padx=5)

        # 状态显示
        br = ttk.Frame(parent); br.pack(fill=tk.X, padx=5, pady=(0,5))
        m1f = ttk.LabelFrame(br, text="电机1状态", padding=3)
        m1f.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0,3))
        self.m1_state_text = tk.Text(m1f, height=8, width=26, state=tk.DISABLED, font=("Consolas",10), bg="#FAFAFA")
        self.m1_state_text.pack(fill=tk.BOTH, expand=True)
        m2f = ttk.LabelFrame(br, text="电机2状态", padding=3)
        m2f.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(3,0))
        self.m2_state_text = tk.Text(m2f, height=8, width=26, state=tk.DISABLED, font=("Consolas",10), bg="#FAFAFA")
        self.m2_state_text.pack(fill=tk.BOTH, expand=True)

        # 动作控制
        af = ttk.LabelFrame(parent, text="动作控制", padding=5)
        af.pack(fill=tk.X, padx=5, pady=(0,5))
        a1 = ttk.Frame(af); a1.pack(fill=tk.X, pady=2)
        ttk.Label(a1, text="点位移动:").pack(side=tk.LEFT)
        self.act_pos_var = tk.StringVar(value="1")
        ttk.Combobox(a1, textvariable=self.act_pos_var, values=["1","2","3","4","5","6","7","8"], width=3, state="readonly").pack(side=tk.LEFT, padx=3)
        self.act_dir_var = tk.BooleanVar(value=False)
        ttk.Radiobutton(a1, text="逆时针", variable=self.act_dir_var, value=False).pack(side=tk.LEFT, padx=2)
        ttk.Radiobutton(a1, text="顺时针", variable=self.act_dir_var, value=True).pack(side=tk.LEFT, padx=2)
        ttk.Button(a1, text="移动", width=5,
                   command=lambda: self._send_cmd(CMD_ACTION_MOVE,
                       bytes([int(self.act_pos_var.get()), 1 if self.act_dir_var.get() else 0]),
                       f"pos={self.act_pos_var.get()}")).pack(side=tk.LEFT, padx=5)
        a2 = ttk.Frame(af); a2.pack(fill=tk.X, pady=2)
        ttk.Label(a2, text="抓取:").pack(side=tk.LEFT)
        ttk.Button(a2, text="▶ 执行", width=6,
                   command=lambda: self._send_cmd(CMD_ACTION_GRAB, b"", "start")).pack(side=tk.LEFT, padx=5)
        ttk.Button(a2, text="■ 停止", width=6,
                   command=lambda: self._send_cmd(CMD_STOP_NOW, bytes([1,0])+bytes([2,0]), "stop")).pack(side=tk.LEFT, padx=5)
        ttk.Separator(a2, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8)
        ttk.Button(a2, text="查询状态", command=self._cmd_query_step_stat).pack(side=tk.LEFT, padx=3)
        ttk.Button(a2, text="回零", command=lambda: self._cmd_reset_cur_pos(self.query_addr_var.get())).pack(side=tk.LEFT, padx=3)
        ttk.Button(a2, text="解除堵转", command=lambda: self._cmd_reset_clog(self.query_addr_var.get())).pack(side=tk.LEFT, padx=3)
        ttk.Button(a2, text="同步触发", command=lambda: self._cmd_sync_motion(self.query_addr_var.get())).pack(side=tk.LEFT, padx=3)

    def _on_motor_sel(self):
        is_m1 = (self._cur_motor.get() == 1)
        self._vel_dir_l1.config(text="正转(逆时针)" if is_m1 else "正转(上升)")
        self._vel_dir_l2.config(text="反转(顺时针)" if is_m1 else "反转(下降)")
        self._pos_dir_l1.config(text="正转(逆时针)" if is_m1 else "正转(上升)")
        self._pos_dir_l2.config(text="反转(顺时针)" if is_m1 else "反转(下降)")

    def _pos_cmd(self):
        addr = self._cur_motor.get()
        d, s, a = self._cur_pos_dir.get(), self._cur_pos_speed.get(), self._cur_pos_acc.get()
        abs_f, sync = self._cur_pos_abs.get(), self._cur_pos_sync.get()
        try: dist_cm = self._cur_pos_dist.get()
        except tk.TclError:
            messagebox.showwarning("参数错误", "请输入有效的距离"); return
        if dist_cm < 0: messagebox.showwarning("参数错误", "距离不能为负数"); return
        dist_val = int(dist_cm * 100)
        self._log("info", f"位置模式: {dist_cm:.1f}cm → 0x78")
        self._cmd_position_cm(addr, d, s, a, dist_val, abs_f, sync)

    # ========== 传感器 & 数据 页 ==========
    def _build_sensor_tab(self, parent: ttk.Frame):
        paned = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        paned.pack(fill=tk.X, padx=5, pady=5)

        # 舵机
        sf = ttk.LabelFrame(paned, text="舵机控制", padding=8)
        paned.add(sf, weight=1)
        ttk.Label(sf, text="通道1 (0-180):").pack(anchor=tk.W)
        self.servo1_var = tk.IntVar(value=0)
        ttk.Scale(sf, from_=0, to=180, variable=self.servo1_var, orient=tk.HORIZONTAL).pack(fill=tk.X)
        sf1 = ttk.Frame(sf); sf1.pack(fill=tk.X, pady=2)
        ttk.Entry(sf1, textvariable=self.servo1_var, width=5).pack(side=tk.LEFT, padx=5)
        ttk.Button(sf1, text="设置", command=lambda: self._cmd_servo(1, self.servo1_var.get())).pack(side=tk.RIGHT)
        ttk.Separator(sf, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=6)
        ttk.Label(sf, text="通道2 (0-180):").pack(anchor=tk.W)
        self.servo2_var = tk.IntVar(value=0)
        ttk.Scale(sf, from_=0, to=180, variable=self.servo2_var, orient=tk.HORIZONTAL).pack(fill=tk.X)
        sf2 = ttk.Frame(sf); sf2.pack(fill=tk.X, pady=2)
        ttk.Entry(sf2, textvariable=self.servo2_var, width=5).pack(side=tk.LEFT, padx=5)
        ttk.Button(sf2, text="设置", command=lambda: self._cmd_servo(2, self.servo2_var.get())).pack(side=tk.RIGHT)

        # 无刷
        bf = ttk.LabelFrame(paned, text="无刷电机", padding=8)
        paned.add(bf, weight=1)
        ttk.Label(bf, text="占空比 (0-100%):").pack(anchor=tk.W)
        self.bldc_var = tk.IntVar(value=50)
        ttk.Scale(bf, from_=0, to=100, variable=self.bldc_var, orient=tk.HORIZONTAL).pack(fill=tk.X)
        brf = ttk.Frame(bf); brf.pack(fill=tk.X, pady=5)
        ttk.Entry(brf, textvariable=self.bldc_var, width=5).pack(side=tk.LEFT, padx=5)
        ttk.Button(brf, text="设置", command=lambda: self._cmd_bldc(self.bldc_var.get())).pack(side=tk.LEFT, padx=5)
        ttk.Button(bf, text="急停", command=self._cmd_bldc_stop).pack(fill=tk.X, pady=(8,0))

        # MPU + OLED
        mf = ttk.LabelFrame(paned, text="MPU6050 & OLED", padding=8)
        paned.add(mf, weight=1)
        ttk.Button(mf, text="校准 MPU", command=self._cmd_mpu_calib).pack(fill=tk.X, pady=1)
        ttk.Button(mf, text="查询姿态", command=self._cmd_query_mpu_att).pack(fill=tk.X, pady=1)
        ttk.Separator(mf, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=4)
        ttk.Label(mf, text="OLED 显示:").pack(anchor=tk.W)
        self.oled_mode_var = tk.StringVar(value="0")
        ttk.Combobox(mf, textvariable=self.oled_mode_var,
            values=["0:关闭","1:MPU姿态","2:电机位置","3:RGB原始","4:RGB滤波","5:RGB占比","6:HSV","7:融合","8:豆子","9:智能"],
            width=14, state="readonly").pack(fill=tk.X, pady=2)
        ttk.Button(mf, text="设置 OLED", command=lambda: self._send_cmd(0x70,
            bytes([int(self.oled_mode_var.get().split(":")[0])]),
            f"OLED={self.oled_mode_var.get()}")).pack(fill=tk.X, pady=1)
        ttk.Separator(mf, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=4)
        self.mpu_stream_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(mf, text="MPU 自动上报", variable=self.mpu_stream_var, command=self._toggle_mpu_stream).pack(anchor=tk.W)
        self.mpu_display = tk.Text(mf, height=5, width=20, state=tk.DISABLED, font=("Consolas",9), bg="#F5F5F5")
        self.mpu_display.pack(fill=tk.BOTH, expand=True, pady=3)

        # 上报行
        pw = ttk.Frame(parent, padding=3); pw.pack(fill=tk.X, padx=5)
        self.pwm_stream_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(pw, text="PWM自动上报", variable=self.pwm_stream_var, command=self._toggle_pwm_stream).pack(side=tk.LEFT)
        self.pwm_status_label = ttk.Label(pw, text=""); self.pwm_status_label.pack(side=tk.LEFT, padx=10)
        self.step_stream_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(pw, text="步进自动上报", variable=self.step_stream_var, command=self._toggle_step_stream).pack(side=tk.LEFT, padx=10)

        # ---- 状态机控制 ----
        sm_frame = ttk.LabelFrame(parent, text="状态机控制", padding=5)
        sm_frame.pack(fill=tk.X, padx=5, pady=(5,0))

        # 模式选择
        sm_top = ttk.Frame(sm_frame); sm_top.pack(fill=tk.X, pady=2)
        self._sm_mode = tk.StringVar(value="manual")
        ttk.Radiobutton(sm_top, text="手动", variable=self._sm_mode, value="manual").pack(side=tk.LEFT, padx=3)
        ttk.Radiobutton(sm_top, text="自动", variable=self._sm_mode, value="auto").pack(side=tk.LEFT, padx=3)

        self._sm_state_label = tk.StringVar(value="状态: IDLE")
        ttk.Label(sm_top, textvariable=self._sm_state_label, foreground="#1565C0").pack(side=tk.RIGHT, padx=5)

        # 三设备到位状态
        sm_devs = ttk.Frame(sm_frame); sm_devs.pack(fill=tk.X, pady=2)
        self._sm_dev_labels = []
        for i, name in enumerate(["左", "中", "右"]):
            lbl = ttk.Label(sm_devs, text=f"{name}:-", foreground="gray")
            lbl.pack(side=tk.LEFT, padx=8)
            self._sm_dev_labels.append(lbl)

        # 按钮
        sm_btns = ttk.Frame(sm_frame); sm_btns.pack(fill=tk.X, pady=2)
        ttk.Button(sm_btns, text="▶ 单步", width=8, command=self._sm_step).pack(side=tk.LEFT, padx=3)
        ttk.Button(sm_btns, text="⏸ 急停", width=8, command=self._sm_estop).pack(side=tk.LEFT, padx=3)
        ttk.Button(sm_btns, text="▶▶ 自动运行", width=10, command=self._sm_auto).pack(side=tk.LEFT, padx=3)

        # 视觉输入 + 路径显示
        sm_vis = ttk.Frame(sm_frame); sm_vis.pack(fill=tk.X, pady=2)
        ttk.Label(sm_vis, text="视觉序列:").pack(side=tk.LEFT)
        self._sm_vision_var = tk.StringVar(value="")
        ttk.Entry(sm_vis, textvariable=self._sm_vision_var, width=8).pack(side=tk.LEFT, padx=3)
        self._sm_path_label = tk.StringVar(value="路径: -")
        ttk.Label(sm_vis, textvariable=self._sm_path_label, foreground="#E65100").pack(side=tk.LEFT, padx=5)

        # 状态机内部变量
        self._sm_seq = 0        # 当前步序号
        self._sm_sent = False   # 当前步指令是否已发
        self._sm_timer = 0      # 消抖计数
        self._sm_checks = {}    # 每步到位检测函数
        self._sm_init()         # 初始化步骤表

        if HAS_MPL:
            self._build_viz_tab(parent)

    # ========== 监视器 ==========
    def _build_monitor_tab(self, parent: ttk.Frame):
        tb = ttk.Frame(parent); tb.pack(fill=tk.X, padx=5, pady=(5,0))
        ttk.Checkbutton(tb, text="自动滚动", variable=self.auto_scroll_var).pack(side=tk.LEFT)
        ttk.Checkbutton(tb, text="显示HEX", variable=self.show_hex_var).pack(side=tk.LEFT, padx=10)
        ttk.Button(tb, text="清空", command=self._clear_monitor).pack(side=tk.LEFT, padx=5)
        self.tx_count_var = tk.StringVar(value="TX: 0")
        self.rx_count_var = tk.StringVar(value="RX: 0")
        self.rx_pkt_count_var = tk.StringVar(value="PKT: 0")
        ttk.Label(tb, textvariable=self.tx_count_var).pack(side=tk.RIGHT, padx=10)
        ttk.Label(tb, textvariable=self.rx_pkt_count_var).pack(side=tk.RIGHT, padx=10)
        ttk.Label(tb, textvariable=self.rx_count_var).pack(side=tk.RIGHT, padx=10)
        self.monitor_text = scrolledtext.ScrolledText(parent, wrap=tk.WORD, font=("Consolas",10),
            bg="#1E1E1E", fg="#D4D4D4", insertbackground="white")
        self.monitor_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        for tag, color in [("tx","#4FC3F7"),("rx","#81C784"),("info","#FFB74D"),("error","#E57373"),("parsed","#CE93D8")]:
            self.monitor_text.tag_config(tag, foreground=color)
        self._tx_count = self._rx_count = self._rx_pkt_count = 0

    # ========== 状态机方法 ==========
    def _sm_init(self):
        """步骤表: [(名称, [(设备id, cmd, data), ...], 检测方式)]"""
        self._sm_steps = [
            ("INIT-校准",    [(0,0x7B,[1]),(1,0x7B,[2]),(2,0x7B,[3])], "flag"),
            ("INIT-上报",    [(0,0x72,[1]),(0,0x73,[1]),(0,0x74,[1]),
                               (1,0x72,[1]),(1,0x73,[1]),(1,0x74,[1]),
                               (2,0x72,[1]),(2,0x73,[1]),(2,0x74,[1])], "none"),
            ("INIT-电机2",   [(0,0x60,[2,1,0]),(1,0x60,[2,1,0]),(2,0x60,[2,1,0]),
                               (0,0x61,[2,0,0x01,0xF4,100,0]),(1,0x61,[2,0,0x01,0xF4,100,0]),(2,0x61,[2,0,0x01,0xF4,100,0])], "stall"),
            ("INIT-停+零",   [(0,0x63,[2,0]),(1,0x63,[2,0]),(2,0x63,[2,0]),
                               (0,0x6A,[2]),(1,0x6A,[2]),(2,0x6A,[2]),
                               (0,0x6B,[2]),(1,0x6B,[2]),(2,0x6B,[2])], "none"),
            ("INIT-舵机",    [(0,0x6C,[1,90]),(1,0x6C,[1,90]),(2,0x6C,[1,90])], "none"),
            ("视觉输入",     [], "vision"),
            ("WAIT_START",   [], "manual"),
            ("MOVE_GRAB",    [(0,0x7A,[6,0]),(1,0x7A,[7,0]),(2,0x7A,[8,1])], "flag"),
            ("GRAB",         [(0,0x79,[]),(1,0x79,[]),(2,0x79,[])], "action_done"),
            ("AVOID1",       [(0,0x7A,[8,0]),(1,0x7A,[1,0]),(2,0x7A,[4,0])], "flag"),
            ("AVOID2",       [(0,0x7A,[5,1]),(1,0x7A,[3,1]),(2,0x7A,[6,1])], "flag"),
            ("DROP",         [], "manual"),
            ("DONE",         [], "none"),
        ]
        self._sm_seq = 0; self._sm_sent = False; self._sm_timer = 0
        self._sm_action_done = [False, False, False]
        self._sm_vision_seq = ""  # 用户输入视觉序列
        self._sm_update_ui()

    def _sm_step(self):
        """单步执行"""
        self._sm_execute()
        self._sm_timer = 0

    def _sm_auto(self):
        """自动运行"""
        self._sm_mode.set("auto")
        self._sm_execute()

    def _sm_estop(self):
        """急停"""
        for d in range(3):
            self._send_cmd_raw(d, 0x63, [1,0])
            self._send_cmd_raw(d, 0x63, [2,0])
            self._send_cmd_raw(d, 0x6E, [])
        self._sm_init()
        self._sm_mode.set("manual")

    def _sm_execute(self):
        """发送当前步指令"""
        name, cmds, detect = self._sm_steps[self._sm_seq]
        if detect == "vision":
            seq = self._sm_vision_var.get().strip()
            if not seq:
                messagebox.showwarning("视觉输入", "请输入视觉序列(如31542)")
                return
            self._sm_vision_seq = seq
        if detect == "manual":
            self._sm_advance()
            return
        if not self._sm_sent:
            self._sm_sent = True
            self._sm_action_done = [False, False, False]
            for dev, cmd, data in cmds:
                self._send_cmd_raw(dev, cmd, data)
            self._log("info", f"[SM] 第{self._sm_seq+1}步: {name} ({len(cmds)}条指令)")
        self._sm_update_ui()

    def _sm_check(self):
        """检查当前步是否完成 (由定时器调用)"""
        if self._sm_seq >= len(self._sm_steps):
            return
        name, cmds, detect = self._sm_steps[self._sm_seq]
        if not self._sm_sent:
            return
        self._sm_timer += 1

        ok = False
        if detect == "none":
            ok = self._sm_timer > 3  # 3 个 tick 的消抖
        elif detect == "flag":
            ok = True
            for d in range(3):
                ms = self.recorders[d].latest_m1
                if ms and not (ms.flag & 0x02):
                    ok = False
        elif detect == "stall":
            ok = True
            for d in range(3):
                ms = self.recorders[d].latest_m1
                if not (ms and (ms.flag & 0x04)):
                    ok = False
        elif detect == "action_done":
            ok = all(self._sm_action_done)
        elif detect == "manual" or detect == "vision":
            return

        if ok:
            self._sm_advance()

    def _sm_advance(self):
        """推进到下一步"""
        self._sm_seq += 1; self._sm_sent = False; self._sm_timer = 0
        self._sm_update_ui()
        if self._sm_mode.get() == "auto" and self._sm_seq < len(self._sm_steps):
            self._sm_execute()

    def _sm_update_ui(self):
        """刷新状态机 UI"""
        if self._sm_seq < len(self._sm_steps):
            name, _, _ = self._sm_steps[self._sm_seq]
            self._sm_state_label.set(f"状态: {self._sm_seq+1}/{len(self._sm_steps)} {name}")
        for i, name in enumerate(["左","中","右"]):
            ms = self.recorders[i].latest_m1
            if ms:
                s = "✅到位" if (ms.flag & 0x02) else ("⚠堵转" if (ms.flag & 0x04) else "⏳...")
            else:
                s = "○离线"
            self._sm_dev_labels[i].config(text=f"{name}:{s}")

    def _send_cmd_raw(self, dev, cmd, data):
        """向指定设备发原始指令 (绕过当前设备选择)"""
        if not self.comm or not self.comm.device_connected(dev):
            return
        full = bytes([cmd]) + bytes(data)
        self.comm.send_packet(full, dev)

    # ========== 可视化 ==========
    def _build_viz_tab(self, parent: ttk.Frame):
        if not HAS_MPL: return
        tb = ttk.Frame(parent, padding=3); tb.pack(fill=tk.X)
        self.viz_record_btn = ttk.Button(tb, text="● 开始记录", width=10, command=self._toggle_recording)
        self.viz_record_btn.pack(side=tk.LEFT, padx=3)
        ttk.Button(tb, text="🗑 清空", width=7, command=self._clear_viz_data).pack(side=tk.LEFT, padx=3)
        self.viz_record_info = tk.StringVar(value="就绪")
        ttk.Label(tb, textvariable=self.viz_record_info, foreground="gray").pack(side=tk.LEFT, padx=10)
        vn = ttk.Notebook(parent); vn.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self._build_motor_viz(vn); self._build_mpu_viz(vn); self._build_other_viz(vn)
        self._start_plot_refresh()

    def _build_motor_viz(self, notebook):
        pg = ttk.Frame(notebook); notebook.add(pg, text="  电机  ")
        sf = ttk.Frame(pg, padding=5); sf.pack(fill=tk.X)
        for i, lb in enumerate(["电机1","电机2"]):
            mf = ttk.LabelFrame(sf, text=lb, padding=5)
            mf.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0,5) if i==0 else (5,0))
            tvars = {k: tk.StringVar(value="-") for k in ('voltage','current','velocity','pos','error','flags')}
            setattr(self, f"_mviz_m{i+1}_vars", tvars)
            for rt, k, u in [("电压:","voltage"," mV"),("电流:","current"," mA"),("转速:","velocity"," RPM"),("位置:","pos",""),("误差:","error",""),("状态:","flags","")]:
                r = ttk.Frame(mf); r.pack(fill=tk.X, pady=1)
                ttk.Label(r, text=rt, width=6).pack(side=tk.LEFT)
                ttk.Label(r, textvariable=tvars[k], foreground="#1565C0").pack(side=tk.LEFT)
                if u: ttk.Label(r, text=u, foreground="gray").pack(side=tk.LEFT)
        if HAS_MPL:
            fig = Figure(figsize=(10,4),dpi=80); self._motor_fig = fig
            gs = fig.add_gridspec(2,1,hspace=0.35)
            self._motor_ax1 = fig.add_subplot(gs[0]); self._motor_ax1.set_ylabel('位置',fontsize=9)
            self._motor_line1_pos, = self._motor_ax1.plot([],[],'b-',lw=1,label='M1')
            self._motor_line2_pos, = self._motor_ax1.plot([],[],'r-',lw=1,label='M2')
            self._motor_ax1.legend(loc='upper right',fontsize=8); self._motor_ax1.grid(True,alpha=0.3)
            self._motor_ax2 = fig.add_subplot(gs[1]); self._motor_ax2.set_ylabel('转速',fontsize=9); self._motor_ax2.set_xlabel('时间(s)',fontsize=9)
            self._motor_line1_vel, = self._motor_ax2.plot([],[],'b-',lw=1,label='M1')
            self._motor_line2_vel, = self._motor_ax2.plot([],[],'r-',lw=1,label='M2')
            self._motor_ax2.legend(loc='upper right',fontsize=8); self._motor_ax2.grid(True,alpha=0.3)
            fig.subplots_adjust(left=0.1,right=0.95,top=0.95,bottom=0.1,hspace=0.35)
            c = FigureCanvasTkAgg(fig,master=pg); c.get_tk_widget().pack(fill=tk.BOTH,expand=True,padx=5,pady=5)
            def sc(event):
                for a in [self._motor_ax1,self._motor_ax2]:
                    xl=a.get_xlim();s=xl[1]-xl[0];ct=event.xdata if event.xdata else xl[0]+s/2
                    a.set_xlim(ct-s*0.4,ct+s*0.4) if event.button=='up' else a.set_xlim(ct-s*0.6,ct+s*0.6)
                c.draw_idle()
            c.mpl_connect('scroll_event',sc)

    def _build_mpu_viz(self, notebook):
        pg = ttk.Frame(notebook); notebook.add(pg, text="  MPU  ")
        df = ttk.Frame(pg,padding=5); df.pack(fill=tk.X)
        self._mpu_roll_var=tk.StringVar(value="-");self._mpu_pitch_var=tk.StringVar(value="-");self._mpu_yaw_var=tk.StringVar(value="-")
        for i,(lb,v) in enumerate([("Roll:",self._mpu_roll_var),("Pitch:",self._mpu_pitch_var),("Yaw:",self._mpu_yaw_var)]):
            ttk.Label(df,text=lb,font=("",11,"bold")).pack(side=tk.LEFT,padx=2)
            ttk.Label(df,textvariable=v,font=("Consolas",14),foreground="#1565C0" if i<2 else "#E65100").pack(side=tk.LEFT,padx=2)
            ttk.Label(df,text="°",foreground="gray").pack(side=tk.LEFT,padx=(0,15))
        if HAS_MPL:
            fig = Figure(figsize=(10,5),dpi=80); self._mpu_fig = fig
            gs = fig.add_gridspec(2,1,hspace=0.35)
            self._mpu_ax1 = fig.add_subplot(gs[0]); self._mpu_ax1.set_ylabel('角度(°)',fontsize=9)
            self._mpu_line_roll, = self._mpu_ax1.plot([],[],'r-',lw=1,label='Roll')
            self._mpu_line_pitch, = self._mpu_ax1.plot([],[],'g-',lw=1,label='Pitch')
            self._mpu_line_yaw, = self._mpu_ax1.plot([],[],'b-',lw=1.5,label='Yaw')
            self._mpu_ax1.legend(loc='upper right',fontsize=8); self._mpu_ax1.grid(True,alpha=0.3)
            self._mpu_ax2 = fig.add_subplot(gs[1]); self._mpu_ax2.set_ylabel('Z加速度',fontsize=9); self._mpu_ax2.set_xlabel('时间(s)',fontsize=9)
            self._mpu_line_z, = self._mpu_ax2.plot([],[],'m-',lw=1,label='Z Accel')
            self._mpu_ax2.legend(loc='upper right',fontsize=8); self._mpu_ax2.grid(True,alpha=0.3)
            fig.subplots_adjust(left=0.1,right=0.95,top=0.95,bottom=0.1,hspace=0.35)
            c = FigureCanvasTkAgg(fig,master=pg); c.get_tk_widget().pack(fill=tk.BOTH,expand=True,padx=5,pady=5)
            def sc(event):
                for a in [self._mpu_ax1,self._mpu_ax2]:
                    xl=a.get_xlim();s=xl[1]-xl[0];ct=event.xdata if event.xdata else xl[0]+s/2
                    a.set_xlim(ct-s*0.4,ct+s*0.4) if event.button=='up' else a.set_xlim(ct-s*0.6,ct+s*0.6)
                c.draw_idle()
            c.mpl_connect('scroll_event',sc)

    def _build_other_viz(self, notebook):
        pg = ttk.Frame(notebook); notebook.add(pg, text="  其他  ")
        cf = ttk.Frame(pg,padding=10); cf.pack(fill=tk.BOTH,expand=True)
        sf=ttk.LabelFrame(cf,text="舵机",padding=10); sf.pack(fill=tk.X,pady=(0,10))
        self._servo1_bar_var=tk.IntVar(value=0);self._servo2_bar_var=tk.IntVar(value=0)
        self._servo1_label=tk.StringVar(value="-");self._servo2_label=tk.StringVar(value="-")
        for lb,bv,lv in [("通道1:",self._servo1_bar_var,self._servo1_label),("通道2:",self._servo2_bar_var,self._servo2_label)]:
            r=ttk.Frame(sf);r.pack(fill=tk.X,pady=3)
            ttk.Label(r,text=lb,width=7).pack(side=tk.LEFT)
            ttk.Progressbar(r,variable=bv,maximum=180,length=200).pack(side=tk.LEFT,padx=5)
            ttk.Label(r,textvariable=lv,width=10).pack(side=tk.LEFT)
        bf=ttk.LabelFrame(cf,text="无刷",padding=10);bf.pack(fill=tk.X,pady=(0,10))
        self._bldc_bar_var=tk.IntVar(value=0);self._bldc_label=tk.StringVar(value="-")
        br=ttk.Frame(bf);br.pack(fill=tk.X,pady=3)
        ttk.Label(br,text="占空比:",width=7).pack(side=tk.LEFT)
        ttk.Progressbar(br,variable=self._bldc_bar_var,maximum=100,length=200).pack(side=tk.LEFT,padx=5)
        ttk.Label(br,textvariable=self._bldc_label,width=10).pack(side=tk.LEFT)
        cf2=ttk.LabelFrame(cf,text="颜色",padding=10);cf2.pack(fill=tk.X)
        self._color_id_label=tk.StringVar(value="未检测")
        self._color_r_label=tk.StringVar(value="R: -");self._color_g_label=tk.StringVar(value="G: -");self._color_b_label=tk.StringVar(value="B: -")
        c1=ttk.Frame(cf2);c1.pack(fill=tk.X,pady=2)
        ttk.Label(c1,text="识别:",font=("",10)).pack(side=tk.LEFT)
        ttk.Label(c1,textvariable=self._color_id_label,font=("",12,"bold"),foreground="#E65100").pack(side=tk.LEFT,padx=10)
        c2=ttk.Frame(cf2);c2.pack(fill=tk.X,pady=2)
        for v in (self._color_r_label,self._color_g_label,self._color_b_label):
            ttk.Label(c2,textvariable=v,font=("Consolas",10)).pack(side=tk.LEFT,padx=10)

    # ========== 连接管理 ==========
    def _refresh_serial_ports(self):
        if not HAS_SERIAL: return
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if hasattr(self, 'serial_port_combo'):
            self.serial_port_combo['values'] = ports
            if ports: self.serial_port_var.set(ports[-1])

    def _create_comm(self) -> CommBase | None:
        idx = self.mode_combo.current()
        if idx == 0: return TcpServerComm(self._on_pkt, self._on_co, self._on_raw, self._on_dev_connect, self._on_dev_disconnect)
        elif idx == 1: return TcpClientComm(self._on_pkt, self._on_co, self._on_raw)
        elif idx == 2: return SerialComm(self._on_pkt, self._on_co, self._on_raw)
        return None

    def _toggle_connect(self):
        if self.comm and self.comm.is_connected:
            self.comm.disconnect(); self.comm = None
            self.connect_btn.config(text=self._get_connect_btn_text())
            self.status_var.set("未连接"); self.status_label.config(foreground="gray")
            return
        if self.comm: self.comm.disconnect(); self.comm = None
        self.comm = self._create_comm()
        if self.comm is None: return
        mi = self.mode_combo.current()
        if mi == 0:
            try: port = int(self.tcp_svr_port_var.get())
            except ValueError: port = 3456
            success = self.comm.connect(port=port)
        elif mi == 1:
            host = self.tcp_cli_host_var.get().strip()
            try: port = int(self.tcp_cli_port_var.get())
            except ValueError: port = 8080
            if not host: messagebox.showwarning("警告", "请输入IP"); return
            success = self.comm.connect(host=host, port=port)
        elif mi == 2:
            port = self.serial_port_var.get().strip()
            try: baud = int(self.serial_baud_var.get())
            except ValueError: baud = 115200
            if not port: messagebox.showwarning("警告", "请选串口"); return
            success = self.comm.connect(port=port, baudrate=baud)
        else: return
        if success:
            self.connect_btn.config(text="断开"); self.status_label.config(foreground="#2E7D32")
        else: self.comm = None

    # ========== 发送命令 ==========
    def _send_cmd(self, cmd: int, data: bytes, desc: str = ""):
        if not self.comm or not self.comm.device_connected(self._current_device):
            self._log("error", f"设备 {self._device_names[self._current_device]} 未连接"); return False
        full = bytes([cmd]) + data
        ok = self.comm.send_packet(full, self._current_device)
        pkt = build_packet(full)
        if ok:
            self._tx_count += 1; self.tx_count_var.set(f"TX: {self._tx_count}")
            self._log("tx", f"发送 {CMD_NAMES.get(cmd,f'0x{cmd:02X}')} {desc} | {format_hex(pkt)}")
        else: self._log("error", f"发送失败: {CMD_NAMES.get(cmd,f'0x{cmd:02X}')}")
        return ok

    def _cmd_enable(self, a, st, sy):
        self._send_cmd(CMD_EN_CONTROL, bytes([a, 1 if st else 0, 1 if sy else 0]), f"addr={a} en={st}")

    def _cmd_velocity(self, a, di, sp, ac, sy):
        sp=max(0,min(3000,sp)); ac=max(0,min(255,ac))
        self._send_cmd(CMD_VEL_CONTROL, bytes([a, 1 if di else 0, (sp>>8)&0xFF, sp&0xFF, ac, 1 if sy else 0]),
                       f"addr={a} vel={sp}")

    def _cmd_position_cm(self, addr, di, sp, ac, dist, ab, sy):
        sp=max(0,min(3000,sp)); ac=max(0,min(255,ac)); dist=max(0,min(0xFFFFFFFF,dist))
        self._send_cmd(CMD_POS_CM, bytes([addr, 1 if di else 0, (sp>>8)&0xFF, sp&0xFF, ac,
                       (dist>>24)&0xFF, (dist>>16)&0xFF, (dist>>8)&0xFF, dist&0xFF, 1 if ab else 0, 1 if sy else 0]),
                       f"addr={addr} dist={dist/100:.1f}cm")

    def _cmd_servo(self, ch, an):
        an=max(0,min(180,an)); self._send_cmd(CMD_SERVO_CONTROL, bytes([ch,an]), f"ch={ch} angle={an}")

    def _cmd_bldc(self, du):
        du=max(0,min(100,du)); self._send_cmd(CMD_BLDC_CONTROL, bytes([du]), f"duty={du}%")

    def _cmd_bldc_stop(self): self._send_cmd(CMD_BLDC_STOP, b"", "")
    def _cmd_mpu_calib(self): self._send_cmd(CMD_MPU_CALIB, b"", "")
    def _cmd_query_mpu_att(self): self._send_cmd(CMD_QUERY_MPU_ATT, b"", "")
    def _cmd_query_step_stat(self): self._send_cmd(CMD_QUERY_STEP_STAT, bytes([self.query_addr_var.get()]), "")
    def _cmd_reset_cur_pos(self, a): self._send_cmd(CMD_RESET_CUR_POS, bytes([a]), f"addr={a}")
    def _cmd_reset_clog(self, a): self._send_cmd(CMD_RESET_CLOG_PRO, bytes([a]), f"addr={a}")
    def _cmd_sync_motion(self, a): self._send_cmd(CMD_SYNC_MOTION, bytes([a]), f"addr={a}")

    def _emergency_stop(self):
        self._send_cmd(CMD_STOP_NOW, bytes([1,0]), "M1 STOP")
        self._send_cmd(CMD_STOP_NOW, bytes([2,0]), "M2 STOP")
        self._cmd_bldc_stop(); self._log("error", "⚠ 急停!"); messagebox.showwarning("急停", "所有电机已停止")

    def _toggle_mpu_stream(self):
        f = 1 if self.mpu_stream_var.get() else 0; self._send_cmd(CMD_MPU_STREAM, bytes([f]), f"{'ON' if f else 'OFF'}")
    def _toggle_step_stream(self):
        f = 1 if self.step_stream_var.get() else 0; self._send_cmd(CMD_STEP_MOTOR_STREAM, bytes([f]), f"{'ON' if f else 'OFF'}")
    def _toggle_pwm_stream(self):
        f = 1 if self.pwm_stream_var.get() else 0; self._send_cmd(CMD_PWM_STATE_STREAM, bytes([f]), f"{'ON' if f else 'OFF'}")

    # ========== 接收 ==========
    def _on_raw(self, raw: bytes):
        self._rx_count += len(raw)
    def _on_co(self, connected: bool, msg: str):
        self.root.after(0, lambda: self._log("info" if "已连接" in msg or "启动" in msg else "error", msg))
    def _on_pkt(self, frame: bytes, data: bytes, device_idx: int = 0):
        self._rx_pkt_count += 1
        self.root.after(0, lambda: self._handle_pkt(frame, data, device_idx))

    def _handle_pkt(self, frame: bytes, data: bytes, device_idx: int):
        self.rx_pkt_count_var.set(f"PKT: {self._rx_pkt_count}")
        self.rx_count_var.set(f"RX: {self._rx_count}")
        rec = self.recorders[device_idx]
        if self.show_hex_var.get(): self._log("rx", f"[D{device_idx+1}] RX {format_hex(frame)}")
        if len(data) < 1: return
        cmd = data[0]; payload = data[1:]; cname = CMD_NAMES.get(cmd, f"0x{cmd:02X}")

        if cmd == CMD_TX_ACK_OK:
            addr = payload[0] if len(payload) > 0 else 0
            self._log("parsed", f"  ↳ {cname} | 电机={addr}")
        elif cmd == CMD_TX_STREAM_MPU:
            if len(payload) >= 6:
                rl = int.from_bytes(payload[0:2], 'little', signed=True)
                pt = int.from_bytes(payload[2:4], 'little', signed=True)
                yw = int.from_bytes(payload[4:6], 'little', signed=True)
                rec.add_mpu(time.time(), rl, pt, yw); self.latest_mpu = (rl, pt, yw)
                if device_idx == self._current_device: self._upd_mpu_disp(rl, pt, yw)
        elif cmd == CMD_TX_STREAM_STEP:
            if len(payload) >= 23:
                ms = MotorState.unpack(payload[:23])
                if ms:
                    rec.add_motor(time.time(), ms)
                    if device_idx == self._current_device: self._upd_motor_disp(ms)
        elif cmd == CMD_TX_STREAM_STATE:
            if len(payload) >= 3:
                s1, s2, bl = payload[0], payload[1], payload[2]
                rec.add_pwm(time.time(), s1, s2, bl)
                self.latest_servo_angles = (s1, s2); self.latest_bldc_duty = bl
                if device_idx == self._current_device:
                    self.pwm_status_label.config(text=f"舵1:{s1}° 舵2:{s2}° BLDC:{bl}%")
        elif cmd == CMD_QUERY_MPU_ATT:
            if len(payload) >= 6:
                rl = int.from_bytes(payload[0:2], 'little', signed=True)
                pt = int.from_bytes(payload[2:4], 'little', signed=True)
                yw = int.from_bytes(payload[4:6], 'little', signed=True)
                rec.add_mpu(time.time(), rl, pt, yw); self.latest_mpu = (rl, pt, yw)
                if device_idx == self._current_device: self._upd_mpu_disp(rl, pt, yw)
                self._log("parsed", f"  ↳ {cname} | R={rl/100:.1f} P={pt/100:.1f} Y={yw/100:.1f}")
        elif cmd == CMD_QUERY_STEP_STAT:
            if len(payload) >= 23:
                ms = MotorState.unpack(payload[:23])
                if ms:
                    rec.add_motor(time.time(), ms)
                    if device_idx == self._current_device: self._upd_motor_disp(ms)
        elif cmd == 0x7C:
            self._sm_action_done[device_idx] = True
            self._log("parsed", f"  ↳ 抓取完成 | 设备{device_idx+1}")
        else:
            self._log("parsed", f"  ↳ {cname} | {format_hex(payload)}")

    def _upd_mpu_disp(self, rl, pt, yw):
        try:
            self.mpu_display.config(state=tk.NORMAL); self.mpu_display.delete(1.0, tk.END)
            self.mpu_display.insert(tk.END, f"  Roll : {rl/100:7.2f}°\n  Pitch: {pt/100:7.2f}°\n  Yaw  : {yw/100:7.2f}°\n  (×0.01)")
            self.mpu_display.config(state=tk.DISABLED)
        except: pass

    def _upd_motor_disp(self, ms: MotorState):
        t = self.m1_state_text if ms.addr == 1 else self.m2_state_text
        if t is None: return
        try:
            t.config(state=tk.NORMAL); t.delete(1.0, tk.END)
            for ln in [f"  电压: {ms.voltage_mv} mV", f"  电流: {ms.phase_current} mA",
                       f"  转速: {ms.velocity} RPM", f"  位置: {ms.current_pos}",
                       f"  误差: {ms.pos_error}", f"  使能: {'✅' if ms.is_enabled else '❌'}",
                       f"  到位: {'✅' if ms.is_in_position else '⏳'}", f"  堵转: {'⚠' if ms.is_stalled else '✅'}"]:
                t.insert(tk.END, ln + "\n")
            t.config(state=tk.DISABLED)
        except: pass

    # ========== 可视化数据刷新 ==========
    def _start_plot_refresh(self):
        self._refresh_plots()
    def _refresh_plots(self):
        self._sm_check()
        if HAS_MPL:
            try: self._upd_motor_plots(); self._upd_mpu_plots(); self._upd_other()
            except: pass
        self._plot_job = self.root.after(200, self._refresh_plots)
        try: self._upd_motor_plots(); self._upd_mpu_plots(); self._upd_other()
        except: pass
        self._plot_job = self.root.after(200, self._refresh_plots)

    def _upd_motor_plots(self):
        r = self.recorder
        for i, ms_var in enumerate([r.latest_m1, r.latest_m2]):
            tvars = getattr(self, f"_mviz_m{i+1}_vars")
            if ms_var:
                tvars['voltage'].set(str(ms_var.voltage_mv)); tvars['current'].set(str(ms_var.phase_current))
                tvars['velocity'].set(str(ms_var.velocity)); tvars['pos'].set(str(ms_var.current_pos))
                tvars['error'].set(str(ms_var.pos_error))
                fl = [];
                if ms_var.is_enabled: fl.append('使能')
                if ms_var.is_in_position: fl.append('到位')
                if ms_var.is_stalled: fl.append('⚠堵转')
                tvars['flags'].set(', '.join(fl) if fl else '-')
        if len(r.timestamps)<2: return
        t0=r.timestamps[0]; t=[ts-t0 for ts in r.timestamps]
        self._motor_line1_pos.set_data(t[:len(r.m1_positions)], list(r.m1_positions))
        self._motor_line2_pos.set_data(t[:len(r.m2_positions)], list(r.m2_positions))
        self._motor_ax1.relim(); self._motor_ax1.autoscale_view()
        self._motor_line1_vel.set_data(t[:len(r.m1_velocities)], list(r.m1_velocities))
        self._motor_line2_vel.set_data(t[:len(r.m2_velocities)], list(r.m2_velocities))
        self._motor_ax2.relim(); self._motor_ax2.autoscale_view()
        if t:
            for a in [self._motor_ax1,self._motor_ax2]:
                if t[-1]>30: a.set_xlim(t[-1]-30,t[-1])
        self._motor_fig.canvas.draw_idle()

    def _upd_mpu_plots(self):
        r=self.recorder
        if len(r.timestamps)<2 or len(r.rolls)<2: return
        t0=r.timestamps[0]; t=[ts-t0 for ts in r.timestamps]
        self._mpu_line_roll.set_data(t[:len(r.rolls)],[v/100.0 for v in r.rolls])
        self._mpu_line_pitch.set_data(t[:len(r.pitches)],[v/100.0 for v in r.pitches])
        self._mpu_line_yaw.set_data(t[:len(r.yaws)],[v/100.0 for v in r.yaws])
        self._mpu_ax1.relim();self._mpu_ax1.autoscale_view()
        self._mpu_line_z.set_data(t[:len(r.z_accels)],list(r.z_accels))
        self._mpu_ax2.relim();self._mpu_ax2.autoscale_view()
        if t:
            for a in [self._mpu_ax1,self._mpu_ax2]:
                if t[-1]>30: a.set_xlim(t[-1]-30,t[-1])
        rl,pt,yw=r.latest_mpu
        self._mpu_roll_var.set(f"{rl/100.0:.2f}");self._mpu_pitch_var.set(f"{pt/100.0:.2f}");self._mpu_yaw_var.set(f"{yw/100.0:.2f}")
        self._mpu_fig.canvas.draw_idle()

    def _upd_other(self):
        r=self.recorder; s1,s2=r.latest_servo
        self._servo1_bar_var.set(s1);self._servo1_label.set(f"{s1}°")
        self._servo2_bar_var.set(s2);self._servo2_label.set(f"{s2}°")
        self._bldc_bar_var.set(r.latest_bldc);self._bldc_label.set(f"{r.latest_bldc}%")

    def _toggle_recording(self):
        r = self.recorder
        if r.is_recording:
            p = r.stop_recording()
            self.viz_record_btn.config(text="● 开始记录")
            self.viz_record_info.set(f"已保存: {os.path.basename(p)}" if p else "就绪")
            self._log("info", f"记录停止, {p}" if p else "停止")
        else:
            r.start_recording()
            self.viz_record_btn.config(text="■ 停止记录"); self.viz_record_info.set("记录中...")

    def _clear_viz_data(self):
        self.recorder.clear(); self.viz_record_info.set("已清空")

    # ========== 日志 ==========
    def _log(self, tag: str, msg: str):
        ts = timestamp(); line = f"[{ts}] {msg}\n"
        self.log_lines.append((tag, line))
        try:
            self.monitor_text.insert(tk.END, line)
            self.monitor_text.tag_add(tag, f"end-{len(line)+1}c", tk.END)
            if self.auto_scroll_var.get(): self.monitor_text.see(tk.END)
        except: pass

    def _clear_monitor(self):
        self.monitor_text.delete(1.0, tk.END)
        self._tx_count = self._rx_count = self._rx_pkt_count = 0
        self.tx_count_var.set("TX: 0"); self.rx_count_var.set("RX: 0"); self.rx_pkt_count_var.set("PKT: 0")

    def _on_close(self):
        if self._plot_job: self.root.after_cancel(self._plot_job); self._plot_job = None
        for rec in self.recorders:
            if rec.is_recording: rec.stop_recording()
        self._save_device_config()
        if self.comm and self.comm.is_connected: self.comm.disconnect()
        self.root.destroy()


# ======================== 主入口 ========================
def main():
    root = tk.Tk()
    app = RosmasterGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()

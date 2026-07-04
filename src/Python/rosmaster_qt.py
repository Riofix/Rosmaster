#!/usr/bin/env python3
"""Rosmaster Qt host console."""

import datetime
import socket
import select
import struct
import sys
import threading
from collections import deque

from PyQt6.QtCore import QObject, Qt, pyqtSignal
from PyQt6.QtGui import QColor, QFont, QPalette
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSlider,
    QSpinBox,
    QTabWidget,
    QTextEdit,
    QToolBar,
    QVBoxLayout,
    QWidget,
)


PACKET_HEADER1 = 0xFF
PACKET_HEADER2_RX = 0xFC
PACKET_HEADER2_TX = 0xFB
PROTOCOL_MAX_DATA_LEN = 64

CMD_NAMES = {
    0x5A: "MPU自动上报",
    0x5B: "步进自动上报",
    0x5C: "PWM状态上报",
    0x5D: "颜色自动上报",
    0x60: "步进使能",
    0x61: "速度控制",
    0x62: "位置控制",
    0x63: "步进急停",
    0x64: "同步触发",
    0x65: "设置回零位",
    0x66: "修改回零参数",
    0x67: "触发回零",
    0x68: "中断回零",
    0x69: "修改控制模式",
    0x6A: "当前位置清零",
    0x6B: "解除堵转",
    0x6C: "舵机控制",
    0x6D: "无刷控制",
    0x6E: "无刷急停",
    0x6F: "颜色传感器",
    0x70: "OLED显示",
    0x71: "MPU校准",
    0x72: "MPU上报开关",
    0x73: "步进上报开关",
    0x74: "PWM上报开关",
    0x75: "颜色上报开关",
    0x76: "全部上报开关",
    0x78: "位置控制(cm)",
    0x79: "启动抓取",
    0x7A: "点位移动",
    0x7B: "校准原点",
    0x7C: "抓取完成",
    0x80: "查询MPU姿态",
    0x81: "查询MPU原始",
    0x82: "查询颜色原始",
    0x83: "查询颜色结果",
    0x84: "查询舵机",
    0x85: "查询无刷",
    0x86: "查询步进状态",
    0x87: "查询步进参数",
    0x90: "ACK",
}

STEP_PARAM_NAMES = {
    0: "地址",
    1: "电压(mV)",
    2: "相电流",
    3: "编码器",
    4: "目标位置",
    5: "速度",
    6: "当前位置",
    7: "位置误差",
    8: "回零状态",
    9: "状态标志",
}


def calc_checksum(pkt_len, data):
    return (pkt_len + sum(data)) & 0xFF


def build_packet(data):
    pkt_len = len(data) + 2
    return bytes([PACKET_HEADER1, PACKET_HEADER2_RX, pkt_len]) + bytes(data) + bytes([calc_checksum(pkt_len, data)])


def fmt_hex(data):
    return " ".join(f"{b:02X}" for b in data)


def ts():
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]


def u16_be(value):
    value = int(value) & 0xFFFF
    return [(value >> 8) & 0xFF, value & 0xFF]


def u32_be(value):
    value = int(value) & 0xFFFFFFFF
    return [(value >> 24) & 0xFF, (value >> 16) & 0xFF, (value >> 8) & 0xFF, value & 0xFF]


def i16_le(data):
    return struct.unpack("<h", bytes(data[:2]))[0]


def i32_le(data):
    return struct.unpack("<i", bytes(data[:4]))[0]


def describe_payload(data):
    if not data:
        return "空包"
    cmd = data[0]
    p = data[1:]
    name = CMD_NAMES.get(cmd, f"0x{cmd:02X}")
    try:
        if cmd == 0x90 and len(p) >= 1:
            return f"{name}: 地址/对象 {p[0]}"
        if cmd == 0x5A and len(p) >= 6:
            return f"{name}: Roll {i16_le(p[0:2]) / 100:.2f}, Pitch {i16_le(p[2:4]) / 100:.2f}, Yaw {i16_le(p[4:6]) / 100:.2f}"
        if cmd in (0x80,) and len(p) >= 6:
            return f"{name}: Roll {i16_le(p[0:2]) / 100:.2f}, Pitch {i16_le(p[2:4]) / 100:.2f}, Yaw {i16_le(p[4:6]) / 100:.2f}"
        if cmd in (0x61, 0x62, 0x78) and len(p) >= 5:
            vel = (p[2] << 8) | p[3]
            return f"{name}: 电机{p[0]}, 方向{p[1]}, 速度{vel}, 加速度{p[4]}"
        if cmd in (0x60, 0x63, 0x65, 0x67, 0x68, 0x6A, 0x6B, 0x86, 0x87):
            return f"{name}: {fmt_hex(p)}"
        if cmd == 0x5B and len(p) >= 23:
            addr = p[0]
            pos = i32_le(p[18:22])
            return f"{name}: 电机{addr}, 当前位置 {pos}"
        if cmd == 0x87 and len(p) in (1, 2, 4):
            return f"{name}: {fmt_hex(p)}"
    except Exception:
        pass
    return f"{name}: {fmt_hex(p)}"


class ProtocolParser:
    def __init__(self, on_packet):
        self._on_packet = on_packet
        self._buf = bytearray()

    def feed(self, raw):
        self._buf.extend(raw)
        while len(self._buf) >= 5:
            if self._buf[0] != PACKET_HEADER1 or self._buf[1] != PACKET_HEADER2_TX:
                self._buf.pop(0)
                continue
            length = self._buf[2]
            total = length + 2
            if len(self._buf) < total:
                break
            frame = bytes(self._buf[:total])
            data = frame[3 : 3 + length - 2]
            if frame[-1] == calc_checksum(length, data):
                self._on_packet(frame, data)
            del self._buf[:total]

    def reset(self):
        self._buf.clear()


class TcpServer(QObject):
    status_changed = pyqtSignal(str)
    raw_received = pyqtSignal(bytes, int)
    packet_received = pyqtSignal(bytes, bytes, int)
    MAX_DEVICES = 3

    def __init__(self):
        super().__init__()
        self._server = None
        self._running = False
        self._socks = [None] * self.MAX_DEVICES
        self._parsers = [
            ProtocolParser(lambda f, d, i=i: self.packet_received.emit(f, d, i)) for i in range(self.MAX_DEVICES)
        ]

    def start(self, port=3456):
        if self._running:
            return True
        try:
            self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._server.settimeout(1.0)
            self._server.bind(("0.0.0.0", int(port)))
            self._server.listen(self.MAX_DEVICES)
            self._running = True
            threading.Thread(target=self._loop, daemon=True).start()
            self.status_changed.emit(f"TCP 监听端口 {port}")
            return True
        except OSError as exc:
            self.status_changed.emit(f"TCP 启动失败: {exc}")
            return False

    def stop(self):
        self._running = False
        for i in range(self.MAX_DEVICES):
            self._close_dev(i, announce=True)
        if self._server:
            try:
                self._server.close()
            except OSError:
                pass
            self._server = None
        self.status_changed.emit("TCP 已停止")

    def send(self, data, dev=0):
        if not (0 <= dev < self.MAX_DEVICES):
            return False
        if self._socks[dev] and self._running:
            try:
                self._socks[dev].sendall(data)
                return True
            except OSError:
                self._close_dev(dev, announce=True)
        return False

    def device_connected(self, dev):
        return 0 <= dev < self.MAX_DEVICES and self._socks[dev] is not None

    def _close_dev(self, idx, announce=False):
        if not (0 <= idx < self.MAX_DEVICES):
            return
        had_sock = self._socks[idx] is not None
        if self._socks[idx]:
            try:
                self._socks[idx].close()
            except OSError:
                pass
            self._socks[idx] = None
        self._parsers[idx].reset()
        if announce and had_sock:
            self.status_changed.emit(f"DISCONNECT:{idx}")

    def _loop(self):
        watch = [self._server]
        while self._running:
            try:
                readable, _, _ = select.select(watch, [], [], 0.5)
            except OSError:
                break
            for sock in readable:
                if sock is self._server:
                    try:
                        cli, addr = self._server.accept()
                        idx = next((i for i, item in enumerate(self._socks) if item is None), -1)
                        if idx < 0:
                            cli.close()
                            continue
                        cli.settimeout(0.1)
                        self._socks[idx] = cli
                        watch.append(cli)
                        self.status_changed.emit(f"CONNECT:{idx}:{addr[0]}")
                    except OSError:
                        pass
                    continue

                idx = next((i for i, item in enumerate(self._socks) if item is sock), -1)
                if idx < 0:
                    continue
                try:
                    data = sock.recv(4096)
                    if data:
                        self.raw_received.emit(data, idx)
                        self._parsers[idx].feed(data)
                    else:
                        if sock in watch:
                            watch.remove(sock)
                        self._close_dev(idx, announce=True)
                except socket.timeout:
                    continue
                except OSError:
                    if sock in watch:
                        watch.remove(sock)
                    self._close_dev(idx, announce=True)


class MonitorPage(QWidget):
    online_changed = pyqtSignal(list)
    packet_decoded = pyqtSignal(int, bytes)

    def __init__(self):
        super().__init__()
        self.server = TcpServer()
        self._online = [False, False, False]
        self._tx_count = 0
        self._rx_count = 0
        self._pkt_count = 0
        self._build()
        self.server.status_changed.connect(self._log_status)
        self.server.raw_received.connect(self._on_raw)
        self.server.packet_received.connect(self._on_pkt)

    def _build(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(10)

        row = QHBoxLayout()
        row.addWidget(QLabel("TCP端口"))
        self.port_edit = QLineEdit("3456")
        self.port_edit.setFixedWidth(84)
        row.addWidget(self.port_edit)
        self.btn_conn = QPushButton("启动监听")
        self.btn_conn.clicked.connect(self.toggle_server)
        row.addWidget(self.btn_conn)
        row.addSpacing(12)
        self.dev_labels = []
        for i in range(3):
            label = QLabel(f"抓手{i + 1} 离线")
            label.setObjectName("offlinePill")
            row.addWidget(label)
            self.dev_labels.append(label)
        row.addStretch()
        layout.addLayout(row)

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Consolas", 10))
        self.log_text.setObjectName("logView")
        layout.addWidget(self.log_text, 1)

        bottom = QHBoxLayout()
        self.tx_label = QLabel("TX 0")
        self.rx_label = QLabel("RX 0 B")
        self.pkt_label = QLabel("PKT 0")
        for label in (self.tx_label, self.rx_label, self.pkt_label):
            label.setObjectName("metricLabel")
            bottom.addWidget(label)
        bottom.addStretch()
        self.cb_scroll = QCheckBox("自动滚动")
        self.cb_scroll.setChecked(True)
        bottom.addWidget(self.cb_scroll)
        clear_btn = QPushButton("清空日志")
        clear_btn.clicked.connect(self.log_text.clear)
        bottom.addWidget(clear_btn)
        layout.addLayout(bottom)

    def toggle_server(self):
        if self.server._running:
            self.server.stop()
            self.btn_conn.setText("启动监听")
            return
        try:
            port = int(self.port_edit.text())
        except ValueError:
            self.append_log("端口必须是数字", "error")
            return
        if self.server.start(port):
            self.btn_conn.setText("停止监听")

    def _log_status(self, msg):
        if msg.startswith("CONNECT:"):
            parts = msg.split(":")
            idx = int(parts[1])
            ip = parts[2] if len(parts) > 2 else ""
            self._online[idx] = True
            self.dev_labels[idx].setText(f"抓手{idx + 1} 在线 {ip}")
            self.dev_labels[idx].setObjectName("onlinePill")
            self.dev_labels[idx].style().unpolish(self.dev_labels[idx])
            self.dev_labels[idx].style().polish(self.dev_labels[idx])
            self.online_changed.emit(self._online[:])
            self.append_log(f"设备{idx + 1} 已连接 {ip}", "status")
        elif msg.startswith("DISCONNECT:"):
            idx = int(msg.split(":")[1])
            self._online[idx] = False
            self.dev_labels[idx].setText(f"抓手{idx + 1} 离线")
            self.dev_labels[idx].setObjectName("offlinePill")
            self.dev_labels[idx].style().unpolish(self.dev_labels[idx])
            self.dev_labels[idx].style().polish(self.dev_labels[idx])
            self.online_changed.emit(self._online[:])
            self.append_log(f"设备{idx + 1} 已断开", "status")
        else:
            self.append_log(msg, "status")

    def _on_raw(self, data, dev):
        self._rx_count += len(data)
        self.rx_label.setText(f"RX {self._rx_count} B")

    def _on_pkt(self, frame, data, dev):
        self._pkt_count += 1
        self.pkt_label.setText(f"PKT {self._pkt_count}")
        desc = describe_payload(data)
        color = "rx"
        if data and data[0] == 0x90:
            color = "ack"
        self.append_log(f"[D{dev + 1}] RX {desc} | {fmt_hex(frame)}", color)
        self.packet_decoded.emit(dev, bytes(data))

    def append_log(self, text, kind="status"):
        colors = {
            "status": "#b7791f",
            "tx": "#1976d2",
            "rx": "#2e7d32",
            "ack": "#00796b",
            "error": "#c62828",
        }
        safe = text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
        self.log_text.append(f'<span style="color:{colors.get(kind, "#333")}">[{ts()}] {safe}</span>')
        if self.cb_scroll.isChecked():
            self.log_text.verticalScrollBar().setValue(self.log_text.verticalScrollBar().maximum())

    def _send_packet_to_dev(self, dev, cmd, payload, desc=""):
        full = bytes([cmd]) + bytes(payload)
        packet = build_packet(full)
        if self.server.send(packet, dev):
            self._tx_count += 1
            self.tx_label.setText(f"TX {self._tx_count}")
            label = CMD_NAMES.get(cmd, f"0x{cmd:02X}")
            suffix = f" {desc}" if desc else ""
            self.append_log(f"[D{dev + 1}] TX {label}{suffix} | {fmt_hex(packet)}", "tx")
            return True
        self.append_log(f"设备{dev + 1} 离线，未发送 {CMD_NAMES.get(cmd, hex(cmd))}", "error")
        return False

    def send_cmd(self, cmd, payload, combo_idx):
        if combo_idx == 0:
            ok = False
            for dev in range(3):
                if self._online[dev]:
                    ok = self._send_packet_to_dev(dev, cmd, payload, "全部") or ok
            if not ok:
                self.append_log("没有在线设备，无法发送全部指令", "error")
            return ok
        dev = combo_idx - 1
        if not (0 <= dev < 3):
            self.append_log("设备选择无效", "error")
            return False
        return self._send_packet_to_dev(dev, cmd, payload)

    def send_dev(self, cmd, payload, dev):
        return self._send_packet_to_dev(dev, cmd, payload)

    def send_online_all(self, cmd, payload, desc=""):
        ok = False
        for dev in range(3):
            if self._online[dev]:
                ok = self._send_packet_to_dev(dev, cmd, payload, desc) or ok
        if not ok:
            self.append_log("没有在线设备，指令未发送", "error")
        return ok

    def online_devices(self):
        return self._online[:]


def make_section(title):
    box = QGroupBox(title)
    box.setObjectName("section")
    return box


def add_labeled(layout, row, col, text, widget):
    layout.addWidget(QLabel(text), row, col)
    layout.addWidget(widget, row, col + 1)


class MotorPanel(QGroupBox):
    def __init__(self, monitor, dev_combo, addr, title, dir_names):
        super().__init__(f"{title}  地址 {addr}")
        self.mon = monitor
        self.dev_combo = dev_combo
        self.addr = addr
        self.dir_names = dir_names
        self.setObjectName("section")
        self._build()

    def _send(self, cmd, payload):
        self.mon.send_cmd(cmd, payload, self.dev_combo.currentIndex())

    def _build(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(10)

        quick = QHBoxLayout()
        self.enable_cb = QCheckBox("使能")
        quick.addWidget(self.enable_cb)
        quick.addWidget(self._button("发送使能", lambda: self._send(0x60, [self.addr, int(self.enable_cb.isChecked()), 0])))
        quick.addWidget(self._button("急停", lambda: self._send(0x63, [self.addr, 0]), "dangerButton"))
        quick.addWidget(self._button("当前位置清零", lambda: self._send(0x6A, [self.addr])))
        quick.addWidget(self._button("解除堵转", lambda: self._send(0x6B, [self.addr])))
        quick.addStretch()
        layout.addLayout(quick)

        tabs = QTabWidget()
        tabs.addTab(self._speed_tab(), "速度")
        tabs.addTab(self._cm_tab(), "位置 cm")
        tabs.addTab(self._pulse_tab(), "位置脉冲")
        tabs.addTab(self._origin_tab(), "回零/模式")
        layout.addWidget(tabs)

    def _speed_tab(self):
        page = QWidget()
        grid = QGridLayout(page)
        self.vel_dir = QComboBox()
        self.vel_dir.addItems(self.dir_names)
        self.vel_speed = QSpinBox()
        self.vel_speed.setRange(0, 3000)
        self.vel_speed.setValue(500)
        self.vel_acc = QSpinBox()
        self.vel_acc.setRange(0, 255)
        self.vel_acc.setValue(100)
        self.vel_sync = QCheckBox("等待同步")
        add_labeled(grid, 0, 0, "方向", self.vel_dir)
        add_labeled(grid, 0, 2, "速度", self.vel_speed)
        add_labeled(grid, 1, 0, "加速度", self.vel_acc)
        grid.addWidget(self.vel_sync, 1, 2)
        grid.addWidget(self._button("运行速度模式", self._send_speed, "primaryButton"), 1, 3)
        return page

    def _cm_tab(self):
        page = QWidget()
        grid = QGridLayout(page)
        self.cm_dir = QComboBox()
        self.cm_dir.addItems(self.dir_names)
        self.cm_dist = QDoubleSpinBox()
        self.cm_dist.setRange(0, 999.99)
        self.cm_dist.setDecimals(2)
        self.cm_dist.setValue(10.0)
        self.cm_speed = QSpinBox()
        self.cm_speed.setRange(0, 3000)
        self.cm_speed.setValue(500)
        self.cm_acc = QSpinBox()
        self.cm_acc.setRange(0, 255)
        self.cm_acc.setValue(100)
        self.cm_abs = QCheckBox("绝对位置")
        self.cm_abs.setChecked(True)
        self.cm_sync = QCheckBox("等待同步")
        add_labeled(grid, 0, 0, "方向", self.cm_dir)
        add_labeled(grid, 0, 2, "距离(cm)", self.cm_dist)
        add_labeled(grid, 1, 0, "速度", self.cm_speed)
        add_labeled(grid, 1, 2, "加速度", self.cm_acc)
        grid.addWidget(self.cm_abs, 2, 0)
        grid.addWidget(self.cm_sync, 2, 1)
        grid.addWidget(self._button("发送 cm 位置", self._send_cm, "primaryButton"), 2, 3)
        return page

    def _pulse_tab(self):
        page = QWidget()
        grid = QGridLayout(page)
        self.pulse_dir = QComboBox()
        self.pulse_dir.addItems(self.dir_names)
        self.pulse_count = QSpinBox()
        self.pulse_count.setRange(0, 99999999)
        self.pulse_count.setValue(1000)
        self.pulse_speed = QSpinBox()
        self.pulse_speed.setRange(0, 3000)
        self.pulse_speed.setValue(500)
        self.pulse_acc = QSpinBox()
        self.pulse_acc.setRange(0, 255)
        self.pulse_acc.setValue(100)
        self.pulse_abs = QCheckBox("绝对位置")
        self.pulse_abs.setChecked(True)
        self.pulse_sync = QCheckBox("等待同步")
        add_labeled(grid, 0, 0, "方向", self.pulse_dir)
        add_labeled(grid, 0, 2, "脉冲", self.pulse_count)
        add_labeled(grid, 1, 0, "速度", self.pulse_speed)
        add_labeled(grid, 1, 2, "加速度", self.pulse_acc)
        grid.addWidget(self.pulse_abs, 2, 0)
        grid.addWidget(self.pulse_sync, 2, 1)
        grid.addWidget(self._button("发送脉冲位置", self._send_pulse, "primaryButton"), 2, 3)
        return page

    def _origin_tab(self):
        page = QWidget()
        grid = QGridLayout(page)
        self.origin_mode = QComboBox()
        self.origin_mode.addItems(["模式0", "模式1", "模式2", "模式3"])
        self.ctrl_mode = QComboBox()
        self.ctrl_mode.addItems(["开环", "闭环"])
        self.save_flag = QCheckBox("保存到电机")
        grid.addWidget(self._button("设置单圈回零位", lambda: self._send(0x65, [self.addr, int(self.save_flag.isChecked())])), 0, 0)
        grid.addWidget(self._button("触发回零", lambda: self._send(0x67, [self.addr, self.origin_mode.currentIndex(), 0])), 0, 1)
        grid.addWidget(self._button("中断回零", lambda: self._send(0x68, [self.addr]), "dangerButton"), 0, 2)
        grid.addWidget(QLabel("控制模式"), 1, 0)
        grid.addWidget(self.ctrl_mode, 1, 1)
        grid.addWidget(self.save_flag, 1, 2)
        grid.addWidget(self._button("写入控制模式", lambda: self._send(0x69, [self.addr, int(self.save_flag.isChecked()), self.ctrl_mode.currentIndex()])), 1, 3)
        grid.addWidget(self._button("查询状态", lambda: self._send(0x86, [self.addr])), 2, 0)
        grid.addWidget(self._button("查询当前位置", lambda: self._send(0x87, [self.addr, 6])), 2, 1)
        grid.addWidget(self._button("同步触发", lambda: self._send(0x64, [self.addr])), 2, 2)
        return page

    def _send_speed(self):
        payload = [self.addr, self.vel_dir.currentIndex()] + u16_be(self.vel_speed.value())
        payload += [self.vel_acc.value(), int(self.vel_sync.isChecked())]
        self._send(0x61, payload)

    def _send_cm(self):
        dist = int(round(self.cm_dist.value() * 100))
        payload = [self.addr, self.cm_dir.currentIndex()] + u16_be(self.cm_speed.value())
        payload += [self.cm_acc.value()] + u32_be(dist)
        payload += [int(self.cm_abs.isChecked()), int(self.cm_sync.isChecked())]
        self._send(0x78, payload)

    def _send_pulse(self):
        payload = [self.addr, self.pulse_dir.currentIndex()] + u16_be(self.pulse_speed.value())
        payload += [self.pulse_acc.value()] + u32_be(self.pulse_count.value())
        payload += [int(self.pulse_abs.isChecked()), int(self.pulse_sync.isChecked())]
        self._send(0x62, payload)

    def _button(self, text, slot, object_name=""):
        button = QPushButton(text)
        if object_name:
            button.setObjectName(object_name)
        button.clicked.connect(slot)
        return button


class MotorPage(QWidget):
    def __init__(self, monitor, dev_combo):
        super().__init__()
        self.mon = monitor
        self.dev_combo = dev_combo
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(12)

        quick = make_section("抓取流程与点位")
        qgrid = QGridLayout(quick)
        self.pos_id = QComboBox()
        self.pos_id.addItems([str(i) for i in range(1, 9)])
        self.pos_dir = QComboBox()
        self.pos_dir.addItems(["逆时针", "顺时针"])
        qgrid.addWidget(QLabel("点位"), 0, 0)
        qgrid.addWidget(self.pos_id, 0, 1)
        qgrid.addWidget(QLabel("方向"), 0, 2)
        qgrid.addWidget(self.pos_dir, 0, 3)
        qgrid.addWidget(self._button("移动到点位", self._move_to_pos, "primaryButton"), 0, 4)
        qgrid.addWidget(self._button("设为原点", self._set_origin), 0, 5)
        self.grab_arm = QCheckBox("解锁抓取")
        self.grab_arm.setToolTip("勾选后才允许发送抓取动作，发送或取消后会自动锁定。")
        self.grab_btn = self._button("启动抓取动作", self._start_grab_checked, "dangerButton")
        self.grab_btn.setEnabled(False)
        self.grab_arm.toggled.connect(self.grab_btn.setEnabled)
        qgrid.addWidget(self.grab_arm, 1, 0)
        qgrid.addWidget(self.grab_btn, 1, 1)
        qgrid.addWidget(self._button("两个步进同步触发", self._sync_both), 1, 2, 1, 2)
        layout.addWidget(quick)

        columns = QHBoxLayout()
        columns.addWidget(MotorPanel(monitor, dev_combo, 1, "电机1 水平/旋转", ["逆时针", "顺时针"]))
        columns.addWidget(MotorPanel(monitor, dev_combo, 2, "电机2 升降", ["上升", "下降"]))
        layout.addLayout(columns, 1)

    def _button(self, text, slot, object_name=""):
        button = QPushButton(text)
        if object_name:
            button.setObjectName(object_name)
        button.clicked.connect(slot)
        return button

    def _move_to_pos(self):
        self.mon.send_cmd(0x7A, [int(self.pos_id.currentText()), self.pos_dir.currentIndex()], self.dev_combo.currentIndex())

    def _set_origin(self):
        reply = QMessageBox.question(self, "确认校准原点", "确定把当前点位写为原点吗？")
        if reply == QMessageBox.StandardButton.Yes:
            self.mon.send_cmd(0x7B, [int(self.pos_id.currentText())], self.dev_combo.currentIndex())

    def _start_grab_checked(self):
        target = self.dev_combo.currentText()
        if self.dev_combo.currentIndex() == 0:
            target = "全部在线抓手"
        reply = QMessageBox.warning(
            self,
            "确认启动抓取",
            f"即将向 {target} 发送抓取动作指令。\n请确认机械结构周围无人手、无障碍物，且抓取起始姿态正确。",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Cancel,
        )
        if reply == QMessageBox.StandardButton.Yes:
            self.mon.send_cmd(0x79, [], self.dev_combo.currentIndex())
        self.grab_arm.setChecked(False)

    def _sync_both(self):
        idx = self.dev_combo.currentIndex()
        self.mon.send_cmd(0x64, [1], idx)
        self.mon.send_cmd(0x64, [2], idx)


class SensorPage(QWidget):
    def __init__(self, monitor, dev_combo):
        super().__init__()
        self.mon = monitor
        self.dev_combo = dev_combo
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(12)

        top = QHBoxLayout()
        top.addWidget(self._servo_box())
        top.addWidget(self._bldc_box())
        top.addWidget(self._mpu_box())
        layout.addLayout(top)
        bottom = QHBoxLayout()
        bottom.addWidget(self._oled_box())
        bottom.addWidget(self._color_box())
        layout.addLayout(bottom)
        layout.addStretch()

    def _servo_box(self):
        box = make_section("舵机")
        lay = QVBoxLayout(box)
        for ch, text in [(0, "广播"), (1, "通道1"), (2, "通道2")]:
            row = QHBoxLayout()
            row.addWidget(QLabel(text))
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(0, 180)
            slider.setValue(90)
            label = QLabel("90°")
            slider.valueChanged.connect(lambda v, lb=label: lb.setText(f"{v}°"))
            button = QPushButton("发送角度")
            button.clicked.connect(lambda _, c=ch, s=slider: self.mon.send_cmd(0x6C, [c, s.value()], self.dev_combo.currentIndex()))
            row.addWidget(slider, 1)
            row.addWidget(label)
            row.addWidget(button)
            lay.addLayout(row)
        query = QPushButton("查询舵机状态")
        query.clicked.connect(lambda: self.mon.send_cmd(0x84, [0], self.dev_combo.currentIndex()))
        lay.addWidget(query)
        return box

    def _bldc_box(self):
        box = make_section("无刷电机")
        lay = QVBoxLayout(box)
        row = QHBoxLayout()
        row.addWidget(QLabel("占空比"))
        self.bldc_slider = QSlider(Qt.Orientation.Horizontal)
        self.bldc_slider.setRange(0, 100)
        self.bldc_slider.setValue(50)
        self.bldc_label = QLabel("50%")
        self.bldc_slider.valueChanged.connect(lambda v: self.bldc_label.setText(f"{v}%"))
        row.addWidget(self.bldc_slider, 1)
        row.addWidget(self.bldc_label)
        lay.addLayout(row)
        buttons = QHBoxLayout()
        run = QPushButton("发送转速")
        run.clicked.connect(lambda: self.mon.send_cmd(0x6D, [self.bldc_slider.value()], self.dev_combo.currentIndex()))
        stop = QPushButton("无刷急停")
        stop.setObjectName("dangerButton")
        stop.clicked.connect(lambda: self.mon.send_cmd(0x6E, [], self.dev_combo.currentIndex()))
        query = QPushButton("查询状态")
        query.clicked.connect(lambda: self.mon.send_cmd(0x85, [], self.dev_combo.currentIndex()))
        buttons.addWidget(run)
        buttons.addWidget(stop)
        buttons.addWidget(query)
        lay.addLayout(buttons)
        return box

    def _mpu_box(self):
        box = make_section("MPU6050")
        lay = QVBoxLayout(box)
        buttons = QGridLayout()
        items = [
            ("校准", 0x71, []),
            ("查询姿态", 0x80, []),
            ("查询原始", 0x81, []),
        ]
        for i, (text, cmd, payload) in enumerate(items):
            btn = QPushButton(text)
            btn.clicked.connect(lambda _, c=cmd, p=payload: self.mon.send_cmd(c, p, self.dev_combo.currentIndex()))
            buttons.addWidget(btn, i // 2, i % 2)
        lay.addLayout(buttons)
        self.mpu_text = QTextEdit()
        self.mpu_text.setReadOnly(True)
        self.mpu_text.setMaximumHeight(120)
        lay.addWidget(self.mpu_text)
        self.mon.packet_decoded.connect(self._on_packet)
        return box

    def _oled_box(self):
        box = make_section("OLED显示")
        lay = QHBoxLayout(box)
        self.oled_mode = QComboBox()
        self.oled_mode.addItems(["0 关闭", "1 MPU", "2 电机", "3 RGB原始", "4 RGB滤波", "5 RGB占比", "6 HSV", "7 融合", "8 豆子", "9 智能"])
        lay.addWidget(self.oled_mode)
        btn = QPushButton("设置OLED")
        btn.clicked.connect(lambda: self.mon.send_cmd(0x70, [self.oled_mode.currentIndex()], self.dev_combo.currentIndex()))
        lay.addWidget(btn)
        return box

    def _color_box(self):
        box = make_section("颜色传感器")
        lay = QGridLayout(box)
        self.color_state = QLabel("未检测")
        self.color_state.setObjectName("strongLabel")
        lay.addWidget(self.color_state, 0, 0, 1, 2)
        enable = QPushButton("开关传感器")
        enable.clicked.connect(lambda: self.mon.send_cmd(0x6F, [1], self.dev_combo.currentIndex()))
        raw = QPushButton("查询原始")
        raw.clicked.connect(lambda: self.mon.send_cmd(0x82, [], self.dev_combo.currentIndex()))
        res = QPushButton("查询结果")
        res.clicked.connect(lambda: self.mon.send_cmd(0x83, [], self.dev_combo.currentIndex()))
        lay.addWidget(enable, 1, 0)
        lay.addWidget(raw, 1, 1)
        lay.addWidget(res, 1, 2)
        return box

    def _on_packet(self, dev, data):
        if not data:
            return
        if data[0] in (0x5A, 0x80) and len(data) >= 7:
            self.mpu_text.setText(describe_payload(data))
        elif data[0] in (0x82, 0x83, 0x5D):
            self.color_state.setText(f"D{dev + 1} {describe_payload(data)}")


class DataPage(QWidget):
    def __init__(self, monitor, dev_combo):
        super().__init__()
        self.mon = monitor
        self.dev_combo = dev_combo
        self.mpu_data = [deque(maxlen=300), deque(maxlen=300), deque(maxlen=300)]
        self.step_data = [deque(maxlen=300) for _ in range(6)]
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(12)

        stream_box = make_section("自动上报")
        stream = QHBoxLayout(stream_box)
        for text, cmd in [("MPU", 0x72), ("步进电机", 0x73), ("PWM状态", 0x74), ("颜色", 0x75)]:
            cb = QCheckBox(text)
            cb.toggled.connect(lambda value, c=cmd: self._set_stream(c, value))
            stream.addWidget(cb)
        stream.addStretch()
        layout.addWidget(stream_box)

        self.status_grid = QGridLayout()
        status_box = make_section("当前数据")
        status_box.setLayout(self.status_grid)
        self.value_labels = {}
        labels = ["MPU", "D1M1", "D1M2", "D2M1", "D2M2", "D3M1", "D3M2", "PWM/颜色"]
        for i, name in enumerate(labels):
            self.status_grid.addWidget(QLabel(name), i, 0)
            label = QLabel("-")
            label.setObjectName("valueLabel")
            self.status_grid.addWidget(label, i, 1)
            self.value_labels[name] = label
        layout.addWidget(status_box)

        self.plot_holder = QWidget()
        plot_layout = QVBoxLayout(self.plot_holder)
        try:
            import pyqtgraph as pg

            self.mpu_plot = pg.PlotWidget(title="MPU姿态角")
            self.mpu_plot.showGrid(x=True, y=True, alpha=0.3)
            self.mpu_curves = [
                self.mpu_plot.plot(pen=(220, 64, 64), name="Roll"),
                self.mpu_plot.plot(pen=(46, 125, 50), name="Pitch"),
                self.mpu_plot.plot(pen=(25, 118, 210), name="Yaw"),
            ]
            self.step_plot = pg.PlotWidget(title="步进当前位置")
            self.step_plot.showGrid(x=True, y=True, alpha=0.3)
            colors = [(216, 67, 21), (239, 108, 0), (0, 137, 123), (67, 160, 71), (57, 73, 171), (94, 53, 177)]
            self.step_curves = [self.step_plot.plot(pen=color, name=f"D{i // 2 + 1}M{i % 2 + 1}") for i, color in enumerate(colors)]
            plot_layout.addWidget(self.mpu_plot)
            plot_layout.addWidget(self.step_plot)
        except Exception:
            plot_layout.addWidget(QLabel("未安装 pyqtgraph，曲线图不可用；当前数据仍会更新。"))
            self.mpu_curves = []
            self.step_curves = []
        layout.addWidget(self.plot_holder, 1)
        self.mon.packet_decoded.connect(self.update_data)

    def _set_stream(self, cmd, enabled):
        combo = self.dev_combo.currentIndex()
        payload = [1 if enabled else 0]
        if combo == 0:
            self.mon.send_online_all(cmd, payload, "上报开关")
        else:
            self.mon.send_cmd(cmd, payload, combo)

    def update_data(self, dev, data):
        if not data:
            return
        cmd = data[0]
        payload = data[1:]
        if cmd in (0x5A, 0x80) and len(payload) >= 6:
            values = [i16_le(payload[0:2]) / 100.0, i16_le(payload[2:4]) / 100.0, i16_le(payload[4:6]) / 100.0]
            for idx, value in enumerate(values):
                self.mpu_data[idx].append(value)
                if self.mpu_curves:
                    self.mpu_curves[idx].setData(list(self.mpu_data[idx]))
            self.value_labels["MPU"].setText(f"D{dev + 1} Roll {values[0]:.2f}  Pitch {values[1]:.2f}  Yaw {values[2]:.2f}")
        elif cmd in (0x5B, 0x86) and len(payload) >= 23:
            addr = payload[0]
            pos = i32_le(payload[18:22])
            idx = dev * 2 + (0 if addr == 1 else 1)
            if 0 <= idx < len(self.step_data):
                self.step_data[idx].append(pos)
                if self.step_curves:
                    self.step_curves[idx].setData(list(self.step_data[idx]))
                self.value_labels[f"D{dev + 1}M{1 if addr == 1 else 2}"].setText(f"当前位置 {pos}")
        elif cmd == 0x87:
            self.value_labels["PWM/颜色"].setText(f"D{dev + 1} 步进参数 {fmt_hex(payload)}")
        elif cmd in (0x5C, 0x5D, 0x84, 0x85):
            self.value_labels["PWM/颜色"].setText(f"D{dev + 1} {describe_payload(data)}")


class StatusPage(QWidget):
    def __init__(self, monitor, dev_combo):
        super().__init__()
        self.mon = monitor
        self.dev_combo = dev_combo
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(12)

        quick = make_section("批量操作")
        grid = QGridLayout(quick)
        actions = [
            ("全部步进使能", self._enable_all),
            ("全部步进脱机", self._disable_all),
            ("全部查询步进", self._query_all_steps),
            ("全部当前位置清零", self._zero_all),
            ("全部解除堵转", self._unclog_all),
            ("全部关闭上报", self._stream_off_all),
        ]
        for idx, (text, slot) in enumerate(actions):
            btn = QPushButton(text)
            btn.clicked.connect(slot)
            grid.addWidget(btn, idx // 3, idx % 3)
        layout.addWidget(quick)

        params = make_section("步进参数查询")
        prow = QHBoxLayout(params)
        self.motor_addr = QComboBox()
        self.motor_addr.addItems(["电机1", "电机2"])
        self.param_id = QComboBox()
        self.param_id.addItems([f"{k} {v}" for k, v in STEP_PARAM_NAMES.items()])
        query = QPushButton("查询参数")
        query.clicked.connect(self._query_param)
        prow.addWidget(QLabel("电机"))
        prow.addWidget(self.motor_addr)
        prow.addWidget(QLabel("参数"))
        prow.addWidget(self.param_id)
        prow.addWidget(query)
        prow.addStretch()
        layout.addWidget(params)
        layout.addStretch()

    def _for_each_selected_device(self, func):
        combo = self.dev_combo.currentIndex()
        if combo == 0:
            for dev, online in enumerate(self.mon.online_devices()):
                if online:
                    func(dev)
        else:
            func(combo - 1)

    def _enable_all(self):
        self._for_each_selected_device(lambda dev: [self.mon.send_dev(0x60, [1, 1, 0], dev), self.mon.send_dev(0x60, [2, 1, 0], dev)])

    def _disable_all(self):
        self._for_each_selected_device(lambda dev: [self.mon.send_dev(0x60, [1, 0, 0], dev), self.mon.send_dev(0x60, [2, 0, 0], dev)])

    def _query_all_steps(self):
        self._for_each_selected_device(lambda dev: [self.mon.send_dev(0x86, [1], dev), self.mon.send_dev(0x86, [2], dev)])

    def _zero_all(self):
        self._for_each_selected_device(lambda dev: [self.mon.send_dev(0x6A, [1], dev), self.mon.send_dev(0x6A, [2], dev)])

    def _unclog_all(self):
        self._for_each_selected_device(lambda dev: [self.mon.send_dev(0x6B, [1], dev), self.mon.send_dev(0x6B, [2], dev)])

    def _stream_off_all(self):
        self._for_each_selected_device(lambda dev: [self.mon.send_dev(cmd, [0], dev) for cmd in (0x72, 0x73, 0x74, 0x75)])

    def _query_param(self):
        addr = self.motor_addr.currentIndex() + 1
        param = int(self.param_id.currentText().split()[0])
        self.mon.send_cmd(0x87, [addr, param], self.dev_combo.currentIndex())


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Rosmaster 上位机控制台")
        self.resize(1280, 820)
        self.monitor = MonitorPage()
        self._build_topbar()
        tabs = QTabWidget()
        tabs.setDocumentMode(True)
        tabs.addTab(self._wrap(MotorPage(self.monitor, self.dev_combo)), "电机与抓取")
        tabs.addTab(self._wrap(SensorPage(self.monitor, self.dev_combo)), "传感器与外设")
        tabs.addTab(self._wrap(DataPage(self.monitor, self.dev_combo)), "数据监测")
        tabs.addTab(self._wrap(StatusPage(self.monitor, self.dev_combo)), "批量调试")
        tabs.addTab(self.monitor, "通信日志")
        self.setCentralWidget(tabs)

    def _build_topbar(self):
        bar = QToolBar("主工具栏")
        bar.setMovable(False)
        bar.addWidget(QLabel("目标设备"))
        self.dev_combo = QComboBox()
        self.dev_combo.addItems(["全部在线", "抓手1", "抓手2", "抓手3"])
        bar.addWidget(self.dev_combo)
        bar.addSeparator()
        start = QPushButton("启动/停止监听")
        start.clicked.connect(self.monitor.toggle_server)
        bar.addWidget(start)
        bar.addSeparator()
        enable = QPushButton("全部使能")
        enable.clicked.connect(self._enable_selected)
        bar.addWidget(enable)
        stop = QPushButton("一键急停")
        stop.setObjectName("dangerButton")
        stop.clicked.connect(self._emergency_stop)
        bar.addWidget(stop)
        self.addToolBar(bar)

    def _wrap(self, widget):
        area = QScrollArea()
        area.setWidgetResizable(True)
        area.setFrameShape(QFrame.Shape.NoFrame)
        area.setWidget(widget)
        return area

    def _selected_devs(self):
        idx = self.dev_combo.currentIndex()
        if idx == 0:
            return [i for i, online in enumerate(self.monitor.online_devices()) if online]
        return [idx - 1]

    def _enable_selected(self):
        for dev in self._selected_devs():
            self.monitor.send_dev(0x60, [1, 1, 0], dev)
            self.monitor.send_dev(0x60, [2, 1, 0], dev)

    def _emergency_stop(self):
        for dev in self._selected_devs():
            self.monitor.send_dev(0x63, [1, 0], dev)
            self.monitor.send_dev(0x63, [2, 0], dev)
            self.monitor.send_dev(0x6E, [], dev)
        self.monitor.append_log("一键急停已触发", "error")


def apply_style(app):
    app.setStyle("Fusion")
    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window, QColor("#f4f6f8"))
    palette.setColor(QPalette.ColorRole.WindowText, QColor("#1f2933"))
    palette.setColor(QPalette.ColorRole.Base, QColor("#ffffff"))
    palette.setColor(QPalette.ColorRole.AlternateBase, QColor("#eef2f6"))
    palette.setColor(QPalette.ColorRole.Text, QColor("#1f2933"))
    palette.setColor(QPalette.ColorRole.Button, QColor("#ffffff"))
    palette.setColor(QPalette.ColorRole.ButtonText, QColor("#1f2933"))
    palette.setColor(QPalette.ColorRole.Highlight, QColor("#2563eb"))
    app.setPalette(palette)
    app.setStyleSheet(
        """
        QWidget {
            font-family: "Microsoft YaHei", "Segoe UI", Arial;
            font-size: 13px;
        }
        QMainWindow, QScrollArea {
            background: #f4f6f8;
        }
        QToolBar {
            background: #ffffff;
            border: 0;
            border-bottom: 1px solid #d8dee6;
            spacing: 8px;
            padding: 8px;
        }
        QGroupBox#section {
            background: #ffffff;
            border: 1px solid #d8dee6;
            border-radius: 6px;
            margin-top: 16px;
            padding: 12px;
            font-weight: 600;
        }
        QGroupBox#section::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 6px;
            color: #27364a;
        }
        QPushButton {
            background: #ffffff;
            border: 1px solid #c8d0da;
            border-radius: 5px;
            padding: 7px 12px;
        }
        QPushButton:hover {
            background: #eef5ff;
            border-color: #8ab4f8;
        }
        QPushButton#primaryButton {
            background: #2563eb;
            border-color: #2563eb;
            color: #ffffff;
            font-weight: 600;
        }
        QPushButton#dangerButton {
            background: #dc2626;
            border-color: #dc2626;
            color: #ffffff;
            font-weight: 700;
        }
        QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox, QTextEdit {
            background: #ffffff;
            border: 1px solid #c8d0da;
            border-radius: 5px;
            padding: 5px;
        }
        QTabWidget::pane {
            border: 0;
        }
        QTabBar::tab {
            background: #e8edf3;
            color: #344054;
            padding: 9px 16px;
            border-top-left-radius: 5px;
            border-top-right-radius: 5px;
            margin-right: 2px;
        }
        QTabBar::tab:selected {
            background: #ffffff;
            color: #1d4ed8;
            font-weight: 600;
        }
        QLabel#onlinePill, QLabel#offlinePill, QLabel#metricLabel {
            border-radius: 5px;
            padding: 5px 10px;
            font-weight: 600;
        }
        QLabel#onlinePill {
            background: #dcfce7;
            color: #166534;
        }
        QLabel#offlinePill {
            background: #eef2f6;
            color: #667085;
        }
        QLabel#metricLabel {
            background: #eef5ff;
            color: #1d4ed8;
        }
        QLabel#strongLabel, QLabel#valueLabel {
            font-weight: 600;
            color: #27364a;
        }
        QTextEdit#logView {
            background: #101828;
            color: #e5e7eb;
            border: 1px solid #1f2937;
        }
        """
    )


if __name__ == "__main__":
    app = QApplication(sys.argv)
    apply_style(app)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())

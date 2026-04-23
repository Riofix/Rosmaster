import sys
import time
import struct
import threading
from collections import deque
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QPushButton, QComboBox, QTextEdit, QCheckBox, QLineEdit, QGridLayout)
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QFont
import pyqtgraph as pg
import serial
import serial.tools.list_ports

# ================= 工业级 PID 控制器 (带斜坡限制) =================
class PositionPID:
    def __init__(self, kp, ki, kd, max_out, max_i, max_accel=1000.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.max_i = max_i
        
        # 最大加速度限制 (决定了速度变化有多平滑)
        self.max_accel = max_accel 
        self.last_output = 0.0 # 记录上一次发出的速度

        self.target = 0.0
        self.error = 0.0
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

    def compute(self, current_pos):
        now = time.time()
        dt = now - self.last_time
        if dt <= 0.0: dt = 0.01

        self.error = self.target - current_pos

        # PID 计算
        p_out = self.kp * self.error
        
        self.integral += self.error * dt
        if self.integral > self.max_i: self.integral = self.max_i
        elif self.integral < -self.max_i: self.integral = -self.max_i
        i_out = self.ki * self.integral

        d_out = self.kd * ((self.error - self.last_error) / dt)

        # 原始需求速度
        raw_output = p_out + i_out + d_out
        if raw_output > self.max_out: raw_output = self.max_out
        elif raw_output < -self.max_out: raw_output = -self.max_out

        # ====== 核心：输出斜坡限制 (加速度控制) ======
        max_change = self.max_accel * dt # 当前周期内允许的最大速度变化量
        
        if raw_output > self.last_output + max_change:
            output = self.last_output + max_change # 想加速太快？强行拉低
        elif raw_output < self.last_output - max_change:
            output = self.last_output - max_change # 想急刹车？强行拉高
        else:
            output = raw_output # 变化平缓，直接放行

        # 更新历史状态
        self.last_output = output
        self.last_error = self.error
        self.last_time = now
        return output

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.last_output = 0.0 # 重置时当前输出速度归零
        self.last_time = time.time()


# ================= 上位机主界面 =================
class RobotMonitor(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot System - PRO MAX (Smooth Profile)")
        self.resize(1280, 900)
        
        self.ser = None
        self.is_running = True
        
        # 核心控制器初始化 (默认加速度设为2000)
        self.pid = PositionPID(kp=0.5, ki=0.0, kd=0.05, max_out=1000, max_i=200, max_accel=2000.0)
        self.pos_loop_enabled = False
        
        self.data_lock = threading.Lock()
        
        self.max_points = 300
        self.data_enc = [deque(maxlen=self.max_points) for _ in range(4)]
        self.data_target = deque(maxlen=self.max_points) 
        self.data_pose = [deque(maxlen=self.max_points) for _ in range(3)]
        self.data_speed = [deque(maxlen=self.max_points) for _ in range(3)]
        self.ui_byte_buffer = deque(maxlen=256) 
        
        self.queue_enc = []
        self.queue_pose = []
        self.queue_speed = []
        self.queue_raw_bytes = []
        self.serial_error_msg = None
        self.latest_voltage = 0.0
        self.ui_loop_counter = 0
        
        self.init_ui()
        self.setup_plots()
        
        # 40ms UI刷新定时器 (25Hz)
        self.ui_timer = QTimer()
        self.ui_timer.timeout.connect(self.update_ui_frame)
        self.ui_timer.start(40) 
        
        threading.Thread(target=self.receive_loop, daemon=True).start()

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # --- 左侧面板 ---
        left_layout = QVBoxLayout()
        main_layout.addLayout(left_layout, 1)

        # 电池
        self.lbl_battery = QLabel("电池电压: --.- V")
        self.lbl_battery.setStyleSheet("font-size: 24px; font-weight: bold; color: #ff5500; background-color: #222; padding: 10px; border-radius: 5px;")
        left_layout.addWidget(self.lbl_battery)
        left_layout.addSpacing(10)

        # 串口
        left_layout.addWidget(QLabel("<b>1. 串口设置</b>"))
        self.port_combo = QComboBox()
        self.refresh_ports()
        left_layout.addWidget(self.port_combo)
        self.btn_connect = QPushButton("连接串口")
        self.btn_connect.clicked.connect(self.toggle_serial)
        left_layout.addWidget(self.btn_connect)
        left_layout.addSpacing(10)

        # PID 控制面板
        left_layout.addWidget(QLabel("<b>2. 整车位置环 (PID + 平滑控制)</b>"))
        
        self.cb_enable_pos = QCheckBox("启用控制 (危险: 请架空小车!)")
        self.cb_enable_pos.setStyleSheet("color: #ff3333; font-weight: bold;")
        self.cb_enable_pos.stateChanged.connect(self.toggle_pid_loop)
        left_layout.addWidget(self.cb_enable_pos)

        grid = QGridLayout()
        grid.addWidget(QLabel("Kp (比例):"), 0, 0)
        self.inp_kp = QLineEdit("0.28")
        grid.addWidget(self.inp_kp, 0, 1)
        
        grid.addWidget(QLabel("Ki (积分):"), 1, 0)
        self.inp_ki = QLineEdit("0.08")
        grid.addWidget(self.inp_ki, 1, 1)
        
        grid.addWidget(QLabel("Kd (微分):"), 2, 0)
        self.inp_kd = QLineEdit("0.1")
        grid.addWidget(self.inp_kd, 2, 1)

        grid.addWidget(QLabel("目标位置:"), 3, 0)
        self.inp_target = QLineEdit("50000")
        grid.addWidget(self.inp_target, 3, 1)

        grid.addWidget(QLabel("最大加速度:"), 4, 0)
        self.inp_accel = QLineEdit("1000")
        grid.addWidget(self.inp_accel, 4, 1)

        left_layout.addLayout(grid)
        
        self.btn_sync_pid = QPushButton("同步参数至控制器")
        self.btn_sync_pid.clicked.connect(self.sync_pid_params)
        left_layout.addWidget(self.btn_sync_pid)
        left_layout.addSpacing(10)

        # 原始数据
        left_layout.addWidget(QLabel("<b>3. 原始数据</b>"))
        self.raw_text_box = QTextEdit()
        self.raw_text_box.setReadOnly(True)
        self.raw_text_box.setFont(QFont("Consolas", 10))
        self.raw_text_box.setStyleSheet("background-color: #1e1e1e; color: #00ff00;")
        left_layout.addWidget(self.raw_text_box)

        # --- 右侧图表 ---
        right_layout = QVBoxLayout()
        main_layout.addLayout(right_layout, 3)
        self.win = pg.GraphicsLayoutWidget()
        right_layout.addWidget(self.win)

    def setup_plots(self):
        pg.setConfigOptions(antialias=False) 
        
        self.p1 = self.win.addPlot(title="编码器数值 & 目标位置")
        self.p1.showGrid(x=True, y=True, alpha=0.3)
        self.p1.addLegend()
        self.curves_enc = [self.p1.plot(pen=(255,100,100), name="M1"), self.p1.plot(pen=(100,255,100), name="M2"),
                           self.p1.plot(pen=(100,100,255), name="M3"), self.p1.plot(pen=(255,255,100), name="M4")]
        self.curve_target = self.p1.plot(pen=pg.mkPen('w', width=2, style=Qt.DashLine), name="Target")
        self.win.nextRow()

        self.p2 = self.win.addPlot(title="解算姿态角 (Roll / Pitch / Yaw)")
        self.p2.showGrid(x=True, y=True, alpha=0.3)
        self.p2.addLegend()
        self.curves_pose = [self.p2.plot(pen=(255,0,0), name="Roll"), self.p2.plot(pen=(0,255,0), name="Pitch"),
                            self.p2.plot(pen=(0,0,255), name="Yaw")]
        self.win.nextRow()

        self.p3 = self.win.addPlot(title="反馈速度 (Vx / Vy / Vz)")
        self.p3.showGrid(x=True, y=True, alpha=0.3)
        self.p3.addLegend()
        self.curves_speed = [self.p3.plot(pen=(255,150,0), name="Vx"), self.p3.plot(pen=(0,255,150), name="Vy"),
                             self.p3.plot(pen=(150,0,255), name="Vz")]

    # ================= 核心发送指令 =================
    def send_speed_command(self, vx=0, vy=0, vz=0, car_type=0x01):
        if not self.ser or not self.ser.is_open: return
        vx = max(min(int(vx), 1000), -1000)
        vy = max(min(int(vy), 1000), -1000)
        vz = max(min(int(vz), 5000), -5000)
        header = b'\xFF\xFC'
        length = 0x0A
        func = 0x12
        payload = struct.pack('<Bhhh', car_type, vx, vy, vz)
        checksum = (length + func + sum(payload)) & 0xFF
        frame = header + bytes([length, func]) + payload + bytes([checksum])
        try:
            self.ser.write(frame)
        except: pass

    # ================= 串口接收与解析 =================
    def receive_loop(self):
        buffer = bytearray()
        while self.is_running:
            if self.ser and self.ser.is_open:
                try:
                    waiting = self.ser.in_waiting
                    if waiting > 0:
                        data = self.ser.read(waiting)
                        buffer.extend(data)
                        self.fast_parse_buffer(buffer)
                        
                        with self.data_lock:
                            self.queue_raw_bytes.extend(data)
                except Exception as e:
                    with self.data_lock: self.serial_error_msg = str(e)
                    try: self.ser.close() 
                    except: pass
                    self.ser = None
            time.sleep(0.005) 

    def fast_parse_buffer(self, buffer):
        while True:
            idx = buffer.find(b'\xff\xfb')
            if idx > 0: del buffer[:idx]
            elif idx == -1:
                if len(buffer) > 1: del buffer[:-1]
                break

            if len(buffer) < 3: break
            total_frame_len = buffer[2] + 2
            if len(buffer) < total_frame_len: break
                
            frame = buffer[:total_frame_len]
            if (sum(frame[2:-1]) & 0xFF) == frame[-1]:
                self.dispatch_payload(frame[3], frame[4:-1])
                del buffer[:total_frame_len] 
            else:
                del buffer[:1] 

    def dispatch_payload(self, func, payload):
        try:
            if func == 0x0A and len(payload) >= 7:
                vx, vy, vz, volt = struct.unpack('<hhhB', payload[:7])
                with self.data_lock: 
                    self.queue_speed.append([vx, vy, vz])
                    self.latest_voltage = volt / 10.0 

            elif func == 0x0C and len(payload) >= 6:
                r, p, y = struct.unpack('<hhh', payload[:6])
                with self.data_lock: 
                    self.queue_pose.append([r/100.0, p/100.0, y/100.0])

            elif func == 0x0D and len(payload) >= 16:
                e1, e2, e3, e4 = struct.unpack('<iiii', payload[:16])
                with self.data_lock: 
                    self.queue_enc.append([e1, e2, e3, e4])
                
                # ====== 核心：事件驱动 PID 计算 ======
                if self.pos_loop_enabled:
                    # 获取 4 轮平均位置 (根据实际情况决定是否需要给某个轮子加负号)
                    avg_pos = (e1 + e2 + e3 + e4) / 4.0
                    
                    with self.data_lock:
                        vx_cmd = self.pid.compute(avg_pos)
                        # 如果误差极小，直接下发0刹车，防止终点微小抖动
                        if abs(self.pid.error) < 50:
                            vx_cmd = 0
                            self.pid.last_output = 0 # 刹停时重置斜坡记录
                    
                    self.send_speed_command(vx=vx_cmd, vy=0, vz=0)

        except struct.error: pass 

    # ================= UI 交互 =================
    def sync_pid_params(self):
        try:
            with self.data_lock:
                self.pid.kp = float(self.inp_kp.text())
                self.pid.ki = float(self.inp_ki.text())
                self.pid.kd = float(self.inp_kd.text())
                self.pid.target = float(self.inp_target.text())
                self.pid.max_accel = float(self.inp_accel.text())
            print(f"参数已同步: Target={self.pid.target}, 加速度={self.pid.max_accel}")
        except ValueError:
            print("参数输入有误，请输入数字。")

    def toggle_pid_loop(self, state):
        self.pos_loop_enabled = (state == 2 or state == Qt.CheckState.Checked)
        if self.pos_loop_enabled:
            self.sync_pid_params()
            with self.data_lock: self.pid.reset()
            print(">>> 位置环控制已启用 <<<")
        else:
            self.send_speed_command(vx=0, vy=0, vz=0)
            print("=== 位置环控制已关闭 (已发送急停) ===")

    def update_ui_frame(self):
        self.ui_loop_counter += 1
        with self.data_lock:
            new_enc = self.queue_enc[:]
            new_pose = self.queue_pose[:]
            new_speed = self.queue_speed[:]
            new_bytes = self.queue_raw_bytes[:]
            err_msg = self.serial_error_msg
            curr_volt = self.latest_voltage
            current_target = self.pid.target 
            
            self.queue_enc.clear()
            self.queue_pose.clear()
            self.queue_speed.clear()
            self.queue_raw_bytes.clear()
            self.serial_error_msg = None

        if err_msg:
            self.raw_text_box.setPlainText(f"!!! 串口断开 !!!\n{err_msg}")
            self.btn_connect.setText("连接串口")
            self.lbl_battery.setText("电池电压: 断开")
            if self.pos_loop_enabled:
                self.cb_enable_pos.setChecked(False) 
            return

        if curr_volt > 0:
            color = "#00ff00" if curr_volt >= 11.1 else "#ff0000" 
            self.lbl_battery.setStyleSheet(f"font-size: 24px; font-weight: bold; color: {color}; background-color: #222; padding: 10px; border-radius: 5px;")
            self.lbl_battery.setText(f"电池电压: {curr_volt:.1f} V")

        if new_enc:
            for vals in new_enc:
                for i in range(4): self.data_enc[i].append(vals[i])
                self.data_target.append(current_target) 
            
            for i in range(4): self.curves_enc[i].setData(list(self.data_enc[i]))
            self.curve_target.setData(list(self.data_target)) 

        if new_pose:
            for vals in new_pose:
                for i in range(3): self.data_pose[i].append(vals[i])
            for i in range(3): self.curves_pose[i].setData(list(self.data_pose[i]))

        if new_speed:
            for vals in new_speed:
                for i in range(3): self.data_speed[i].append(vals[i])
            for i in range(3): self.curves_speed[i].setData(list(self.data_speed[i]))

        if new_bytes: self.ui_byte_buffer.extend(new_bytes)
        if self.ui_loop_counter >= 5:
            self.ui_loop_counter = 0
            if len(self.ui_byte_buffer) > 0:
                hex_list = [f"{b:02X}" for b in self.ui_byte_buffer]
                lines = [" ".join(hex_list[i:i+16]) for i in range(0, len(hex_list), 16)]
                self.raw_text_box.setPlainText('\n'.join(lines))
                scrollbar = self.raw_text_box.verticalScrollBar()
                scrollbar.setValue(scrollbar.maximum())

    def toggle_serial(self):
        if self.ser and self.ser.is_open:
            self.cb_enable_pos.setChecked(False) 
            self.ser.close()
            self.btn_connect.setText("连接串口")
        else:
            try:
                port = self.port_combo.currentText()
                if port:
                    self.ser = serial.Serial(port, 115200, timeout=0.01)
                    self.btn_connect.setText("断开连接")
                    self.raw_text_box.clear()
            except Exception as e:
                self.raw_text_box.setPlainText(f"无法连接: {e}")

    def refresh_ports(self):
        self.port_combo.clear()
        self.port_combo.addItems([p.device for p in serial.tools.list_ports.comports()])

if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = RobotMonitor()
    gui.show()
    sys.exit(app.exec())
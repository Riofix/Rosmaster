#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import threading
import time
from .protocol_parser import RosmasterProtocol

class ChassisDriver:
    """
    Rosmaster 底盘串口驱动层
    负责串口生命周期管理、线程安全的读写、以及处理串口粘包/半包问题。
    不包含任何 ROS 相关代码，完全解耦。
    """
    def __init__(self, port='/dev/rosmaster', baudrate=115200, data_callback=None):
        self.port = port
        self.baudrate = baudrate
        self.data_callback = data_callback  # 接收到合法数据时的回调函数
        
        self.serial = None
        self.is_running = False
        self.read_thread = None
        self.buffer = bytearray()
        self.lock = threading.Lock() # 用于线程安全的发送

    def connect(self):
        """连接串口并启动接收线程"""
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.1)
            if self.serial.is_open:
                self.is_running = True
                self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
                self.read_thread.start()
                print(f"[Driver] Successfully connected to {self.port} at {self.baudrate} baud.")
                return True
        except serial.SerialException as e:
            print(f"[Driver] Failed to connect to {self.port}: {e}")
            return False

    def disconnect(self):
        """断开串口并停止接收线程"""
        self.is_running = False
        if self.read_thread:
            self.read_thread.join(timeout=1.0)
        if self.serial and self.serial.is_open:
            self.serial.close()
        print("[Driver] Disconnected.")

    def _read_loop(self):
        """后台读取线程：处理不定长的字节流以及粘包/半包"""
        while self.is_running and self.serial.is_open:
            try:
                # 检查是否有数据可读
                waiting = self.serial.in_waiting
                if waiting > 0:
                    raw_data = self.serial.read(waiting)
                    self.buffer.extend(raw_data)
                    self._process_buffer()
                else:
                    time.sleep(0.005) # 避免 CPU 占用过高
            except Exception as e:
                print(f"[Driver] Read error: {e}")
                time.sleep(0.1)

    def _process_buffer(self):
        """
        核心逻辑：从缓冲区中提取完整的协议帧
        帧结构: [0xFF, 0xFB, 长度, 功能字, 数据..., 校验位]
        """
        while True:
            # 1. 寻找接收包头 0xFF 0xFB
            idx = self.buffer.find(RosmasterProtocol.HEADER_RECV)
            
            if idx == -1:
                # 没找到包头。如果缓冲区的最后一个字节是 0xFF，保留它（可能是包头的一半），否则清空垃圾数据
                if len(self.buffer) > 0 and self.buffer[-1] == 0xFF:
                    self.buffer = self.buffer[-1:]
                else:
                    self.buffer.clear()
                break
            
            # 2. 丢弃包头之前的垃圾数据
            if idx > 0:
                self.buffer = self.buffer[idx:]
            
            # 3. 检查是否有足够的数据读取长度字节（下标为2）
            if len(self.buffer) < 3:
                break # 等待更多数据
                
            # 4. 计算整个帧的预期总长度
            # 根据协议：长度计算是除包头外的所有字节。所以总长度 = 包头(2) + 长度字节的值
            expected_length = self.buffer[2]
            total_frame_size = expected_length + 2
            
            # 5. 检查缓冲区是否已经收到了完整的帧
            if len(self.buffer) < total_frame_size:
                break # 属于半包，退出循环等待下一次读取补齐
                
            # 6. 提取完整的一帧，并从缓冲区中移除
            frame_bytes = bytes(self.buffer[:total_frame_size])
            self.buffer = self.buffer[total_frame_size:]
            
            # 7. 交给协议层解析
            parsed_data = RosmasterProtocol.unpack_frame(frame_bytes)
            
            # 8. 如果解析成功且没有错误，触发回调通知上层
            if "error" not in parsed_data:
                if self.data_callback:
                    self.data_callback(parsed_data)
            else:
                # 如果校验失败，可能是假包头，打印警告并继续解析后续缓冲区
                # print(f"[Driver] Parse error: {parsed_data['error']}")
                pass

    def send_raw(self, cmd_bytes):
        """线程安全的底层发送方法"""
        if self.is_running and self.serial and self.serial.is_open:
            with self.lock:
                self.serial.write(cmd_bytes)
                self.serial.flush()

    # =====================================================================
    # 提供给上层 (ROS node) 调用的便捷控制接口 (直接调用 protocol_parser)
    # =====================================================================

    def set_velocity(self, linear_x, linear_y, angular_z, car_type=1):
        """控制小车底盘速度"""
        cmd = RosmasterProtocol.cmd_set_velocity(car_type, linear_x, linear_y, angular_z)
        self.send_raw(cmd)

    def set_pwm_servo(self, servo_id, angle):
        """控制单个PWM舵机"""
        cmd = RosmasterProtocol.cmd_set_pwm_servo(servo_id, angle)
        self.send_raw(cmd)

    def set_beep(self, time_ms):
        """控制蜂鸣器"""
        cmd = RosmasterProtocol.cmd_set_beep(time_ms)
        self.send_raw(cmd)

    def set_rgb_light(self, led_id, r, g, b):
        """控制RGB灯带"""
        cmd = RosmasterProtocol.cmd_set_rgb_light(led_id, r, g, b)
        self.send_raw(cmd)
        
    def request_motion_data(self):
        """手动请求里程计/电压数据（如果关闭了自动发送）"""
        cmd = RosmasterProtocol.req_motion_data()
        self.send_raw(cmd)

    # ... 你可以根据需要，像上面这样继续把 protocol_parser 中的指令包成简单方法
    # 也可以在外部直接调用 RosmasterProtocol 打包好后，传入 send_raw(cmd)
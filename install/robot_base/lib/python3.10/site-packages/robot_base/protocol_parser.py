#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import struct

class RosmasterProtocol:
    """
    Rosmaster 底板纯净通信协议解析层
    完全基于小端(Little-Endian)字节序处理
    无任何串口或 ROS 依赖，方便进行单元测试
    """
    HEADER_SEND = b'\xFF\xFC'
    HEADER_RECV = b'\xFF\xFB'

    # ==========================================
    # 基础校验与打包解包核心函数
    # ==========================================
    @staticmethod
    def calc_checksum(frame_bytes):
        """计算校验和: 从长度位(下标2)累加到校验位前，再对256取余"""
        return sum(frame_bytes[2:]) % 256

    @staticmethod
    def pack_frame(func_code, data_bytes=b''):
        """
        协议通用打包函数
        长度 = 长度位(1) + 功能字(1) + 数据长度 + 校验位(1) = len(data_bytes) + 3
        """
        length = len(data_bytes) + 3
        frame = bytearray(RosmasterProtocol.HEADER_SEND)
        frame.append(length)
        frame.append(func_code)
        frame.extend(data_bytes)
        frame.append(RosmasterProtocol.calc_checksum(frame))
        return bytes(frame)

    @staticmethod
    def unpack_frame(frame):
        """
        协议通用解包函数
        输入一个完整的帧字节流，验证包头和校验和，解析出核心数据
        返回: dict
        """
        if len(frame) < 4:
            return {"error": "Frame too short"}
        if not frame.startswith(RosmasterProtocol.HEADER_RECV):
            return {"error": "Invalid Header"}
        
        length = frame[2]
        if len(frame) < length + 2:
            return {"error": "Incomplete Frame"}
        
        expected_checksum = RosmasterProtocol.calc_checksum(frame[:length + 1])
        actual_checksum = frame[length + 1]
        if expected_checksum != actual_checksum:
            return {"error": f"Checksum mismatch. Expected {expected_checksum}, got {actual_checksum}"}

        func_code = frame[3]
        data = frame[4: length + 1]
        return RosmasterProtocol._parse_payload(func_code, data)

    # ==========================================
    # 发送指令生成函数 (Host -> Board)
    # ==========================================
    @staticmethod
    def cmd_set_auto_report(enable=True, save_flash=False):
        """设置自动发送数据 (0x01)"""
        switch = 0x01 if enable else 0x00
        save = 0x5F if save_flash else 0x00
        return RosmasterProtocol.pack_frame(0x01, struct.pack('<B B', switch, save))

    @staticmethod
    def cmd_set_beep(time_ms):
        """蜂鸣器 (0x02) time_ms: =0关闭, =1长响, >=10响xx毫秒"""
        return RosmasterProtocol.pack_frame(0x02, struct.pack('<H', time_ms))

    @staticmethod
    def cmd_set_pwm_servo(servo_id, angle):
        """单路PWM舵机控制 (0x03) ID:1~4, Angle:0~180"""
        return RosmasterProtocol.pack_frame(0x03, struct.pack('<B B', servo_id, angle))

    @staticmethod
    def cmd_set_all_pwm_servos(s1, s2, s3, s4):
        """一次控制所有PWM舵机 (0x04)"""
        return RosmasterProtocol.pack_frame(0x04, struct.pack('<B B B B', s1, s2, s3, s4))

    @staticmethod
    def cmd_set_rgb_light(led_id, r, g, b):
        """彩色灯带控制 (0x05) led_id:0~14, 0xff控制所有"""
        return RosmasterProtocol.pack_frame(0x05, struct.pack('<B B B B', led_id, r, g, b))

    @staticmethod
    def cmd_set_rgb_effect(effect, speed=0xff, param=0xff):
        """彩色灯带特效 (0x06) effect:0-6"""
        return RosmasterProtocol.pack_frame(0x06, struct.pack('<B B B', effect, speed, param))

    @staticmethod
    def cmd_set_motor_pwm(m1, m2, m3, m4):
        """控制电机PWM速度，无编码器 (0x10) 范围：±100"""
        return RosmasterProtocol.pack_frame(0x10, struct.pack('<b b b b', m1, m2, m3, m4))

    @staticmethod
    def cmd_set_car_state(car_type, state, speed):
        """小车状态控制 (0x11) state: 0停车 1前进 2后退 3左移 4右移 5左旋 6右旋 7刹车"""
        return RosmasterProtocol.pack_frame(0x11, struct.pack('<B B B B', car_type, state, speed, 0x00))

    @staticmethod
    def cmd_set_velocity(car_type, linear_x, linear_y, angular_z):
        """小车运动控制 (0x12) 发送的速度会被放大1000倍传输"""
        vx = int(linear_x * 1000)
        vy = int(linear_y * 1000)
        vz = int(angular_z * 1000)
        return RosmasterProtocol.pack_frame(0x12, struct.pack('<B h h h', car_type, vx, vy, vz))

    @staticmethod
    def cmd_set_pid(p, i, d, save_flash=False):
        """小车速度PID调节 (0x13) 参数自动放大1000倍"""
        save = 0x5F if save_flash else 0x00
        return RosmasterProtocol.pack_frame(0x13, struct.pack('<h h h B', int(p*1000), int(i*1000), int(d*1000), save))

    @staticmethod
    def cmd_set_yaw_pid(p, i, d, save_flash=False):
        """偏航角PID调节 (0x14)"""
        save = 0x5F if save_flash else 0x00
        return RosmasterProtocol.pack_frame(0x14, struct.pack('<h h h B', int(p*1000), int(i*1000), int(d*1000), save))

    @staticmethod
    def cmd_set_car_type(car_type):
        """设置小车类型 (0x15) 1:X3, 2:X3PLUS, 4:X1, 5:R2"""
        return RosmasterProtocol.pack_frame(0x15, struct.pack('<B B', car_type, 0x5F))

    @staticmethod
    def cmd_set_bus_servo(servo_id, position, time_ms):
        """总线舵机控制 (0x20) ID:1-250 (0xFE所有), 位置:96-4000"""
        return RosmasterProtocol.pack_frame(0x20, struct.pack('<B H H', servo_id, position, time_ms))

    @staticmethod
    def cmd_set_bus_servo_id(servo_id):
        """设置总线舵机ID (0x21)"""
        return RosmasterProtocol.pack_frame(0x21, struct.pack('<B', servo_id))

    @staticmethod
    def cmd_set_bus_servo_torque(enable=True):
        """设置总线舵机扭矩力 (0x22)"""
        return RosmasterProtocol.pack_frame(0x22, struct.pack('<B', 0x01 if enable else 0x00))

    @staticmethod
    def cmd_set_arm_joints(j1, j2, j3, j4, j5, j6, time_ms):
        """控制机械臂关节 (0x23) 传6个位置和时间"""
        return RosmasterProtocol.pack_frame(0x23, struct.pack('<H H H H H H H', j1, j2, j3, j4, j5, j6, time_ms))

    @staticmethod
    def cmd_calibrate_arm_joint(servo_id):
        """机械臂校准 (0x24)"""
        return RosmasterProtocol.pack_frame(0x24, struct.pack('<B', servo_id))

    @staticmethod
    def cmd_factory_reset():
        """清空flash数据 / 恢复出厂设置 (0xA0)"""
        return RosmasterProtocol.pack_frame(0xA0, struct.pack('<B', 0x5F))

    # ==========================================
    # 数据请求指令生成 (手动请求时使用)
    # ==========================================
    @staticmethod
    def request_data(param1, param2=0x00):
        """通用的数据请求结构 (0x50)"""
        return RosmasterProtocol.pack_frame(0x50, struct.pack('<B B', param1, param2))

    @staticmethod
    def req_firmware_version(): return RosmasterProtocol.request_data(0x51)
    @staticmethod
    def req_motion_data(): return RosmasterProtocol.request_data(0x0A)
    @staticmethod
    def req_imu_data(): return RosmasterProtocol.request_data(0x0B)
    @staticmethod
    def req_attitude_angle(): return RosmasterProtocol.request_data(0x0C)
    @staticmethod
    def req_bus_servo_position(servo_id): return RosmasterProtocol.request_data(0x20, servo_id)
    @staticmethod
    def req_arm_all_joints(): return RosmasterProtocol.request_data(0x23, 0x01)
    @staticmethod
    def req_pid(): return RosmasterProtocol.request_data(0x13, 0x01)
    @staticmethod#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import struct

class RosmasterProtocol:
    """
    Rosmaster 底板纯净通信协议解析层
    完全基于小端(Little-Endian)字节序处理
    无任何串口或 ROS 依赖，方便进行单元测试
    """
    HEADER_SEND = b'\xFF\xFC'
    HEADER_RECV = b'\xFF\xFB'

    # ==========================================
    # 基础校验与打包解包核心函数
    # ==========================================
    @staticmethod
    def calc_checksum(frame_bytes):
        """计算校验和: 从长度位(下标2)累加到校验位前，再对256取余"""
        return sum(frame_bytes[2:]) % 256

    @staticmethod
    def pack_frame(func_code, data_bytes=b''):
        """
        协议通用打包函数
        长度 = 长度位(1) + 功能字(1) + 数据长度 + 校验位(1) = len(data_bytes) + 3
        """
        length = len(data_bytes) + 3
        frame = bytearray(RosmasterProtocol.HEADER_SEND)
        frame.append(length)
        frame.append(func_code)
        frame.extend(data_bytes)
        frame.append(RosmasterProtocol.calc_checksum(frame))
        return bytes(frame)

    @staticmethod
    def unpack_frame(frame):
        """
        协议通用解包函数
        输入一个完整的帧字节流，验证包头和校验和，解析出核心数据
        返回: dict
        """
        if len(frame) < 4:
            return {"error": "Frame too short"}
        if not frame.startswith(RosmasterProtocol.HEADER_RECV):
            return {"error": "Invalid Header"}
        
        length = frame[2]
        if len(frame) < length + 2:
            return {"error": "Incomplete Frame"}
        
        expected_checksum = RosmasterProtocol.calc_checksum(frame[:length + 1])
        actual_checksum = frame[length + 1]
        if expected_checksum != actual_checksum:
            return {"error": f"Checksum mismatch. Expected {expected_checksum}, got {actual_checksum}"}

        func_code = frame[3]
        data = frame[4: length + 1]
        return RosmasterProtocol._parse_payload(func_code, data)

    # ==========================================
    # 发送指令生成函数 (Host -> Board)
    # ==========================================
    @staticmethod
    def cmd_set_auto_report(enable=True, save_flash=False):
        """设置自动发送数据 (0x01)"""
        switch = 0x01 if enable else 0x00
        save = 0x5F if save_flash else 0x00
        return RosmasterProtocol.pack_frame(0x01, struct.pack('<B B', switch, save))

    @staticmethod
    def cmd_set_beep(time_ms):
        """蜂鸣器 (0x02) time_ms: =0关闭, =1长响, >=10响xx毫秒"""
        return RosmasterProtocol.pack_frame(0x02, struct.pack('<H', time_ms))

    @staticmethod
    def cmd_set_pwm_servo(servo_id, angle):
        """单路PWM舵机控制 (0x03) ID:1~4, Angle:0~180"""
        return RosmasterProtocol.pack_frame(0x03, struct.pack('<B B', servo_id, angle))

    @staticmethod
    def cmd_set_all_pwm_servos(s1, s2, s3, s4):
        """一次控制所有PWM舵机 (0x04)"""
        return RosmasterProtocol.pack_frame(0x04, struct.pack('<B B B B', s1, s2, s3, s4))

    @staticmethod
    def cmd_set_rgb_light(led_id, r, g, b):
        """彩色灯带控制 (0x05) led_id:0~14, 0xff控制所有"""
        return RosmasterProtocol.pack_frame(0x05, struct.pack('<B B B B', led_id, r, g, b))

    @staticmethod
    def cmd_set_rgb_effect(effect, speed=0xff, param=0xff):
        """彩色灯带特效 (0x06) effect:0-6"""
        return RosmasterProtocol.pack_frame(0x06, struct.pack('<B B B', effect, speed, param))

    @staticmethod
    def cmd_set_motor_pwm(m1, m2, m3, m4):
        """控制电机PWM速度，无编码器 (0x10) 范围：±100"""
        return RosmasterProtocol.pack_frame(0x10, struct.pack('<b b b b', m1, m2, m3, m4))

    @staticmethod
    def cmd_set_car_state(car_type, state, speed):
        """小车状态控制 (0x11) state: 0停车 1前进 2后退 3左移 4右移 5左旋 6右旋 7刹车"""
        return RosmasterProtocol.pack_frame(0x11, struct.pack('<B B B B', car_type, state, speed, 0x00))

    @staticmethod
    def cmd_set_velocity(car_type, linear_x, linear_y, angular_z):
        """小车运动控制 (0x12) 发送的速度会被放大1000倍传输"""
        vx = int(linear_x * 1000)
        vy = int(linear_y * 1000)
        vz = int(angular_z * 1000)
        return RosmasterProtocol.pack_frame(0x12, struct.pack('<B h h h', car_type, vx, vy, vz))

    @staticmethod
    def cmd_set_pid(p, i, d, save_flash=False):
        """小车速度PID调节 (0x13) 参数自动放大1000倍"""
        save = 0x5F if save_flash else 0x00
        return RosmasterProtocol.pack_frame(0x13, struct.pack('<h h h B', int(p*1000), int(i*1000), int(d*1000), save))

    @staticmethod
    def cmd_set_yaw_pid(p, i, d, save_flash=False):
        """偏航角PID调节 (0x14)"""
        save = 0x5F if save_flash else 0x00
        return RosmasterProtocol.pack_frame(0x14, struct.pack('<h h h B', int(p*1000), int(i*1000), int(d*1000), save))

    @staticmethod
    def cmd_set_car_type(car_type):
        """设置小车类型 (0x15) 1:X3, 2:X3PLUS, 4:X1, 5:R2"""
        return RosmasterProtocol.pack_frame(0x15, struct.pack('<B B', car_type, 0x5F))

    @staticmethod
    def cmd_set_bus_servo(servo_id, position, time_ms):
        """总线舵机控制 (0x20) ID:1-250 (0xFE所有), 位置:96-4000"""
        return RosmasterProtocol.pack_frame(0x20, struct.pack('<B H H', servo_id, position, time_ms))

    @staticmethod
    def cmd_set_bus_servo_id(servo_id):
        """设置总线舵机ID (0x21)"""
        return RosmasterProtocol.pack_frame(0x21, struct.pack('<B', servo_id))

    @staticmethod
    def cmd_set_bus_servo_torque(enable=True):
        """设置总线舵机扭矩力 (0x22)"""
        return RosmasterProtocol.pack_frame(0x22, struct.pack('<B', 0x01 if enable else 0x00))

    @staticmethod
    def cmd_set_arm_joints(j1, j2, j3, j4, j5, j6, time_ms):
        """控制机械臂关节 (0x23) 传6个位置和时间"""
        return RosmasterProtocol.pack_frame(0x23, struct.pack('<H H H H H H H', j1, j2, j3, j4, j5, j6, time_ms))

    @staticmethod
    def cmd_calibrate_arm_joint(servo_id):
        """机械臂校准 (0x24)"""
        return RosmasterProtocol.pack_frame(0x24, struct.pack('<B', servo_id))

    @staticmethod
    def cmd_factory_reset():
        """清空flash数据 / 恢复出厂设置 (0xA0)"""
        return RosmasterProtocol.pack_frame(0xA0, struct.pack('<B', 0x5F))

    # ==========================================
    # 数据请求指令生成 (手动请求时使用)
    # ==========================================
    @staticmethod
    def request_data(param1, param2=0x00):
        """通用的数据请求结构 (0x50)"""
        return RosmasterProtocol.pack_frame(0x50, struct.pack('<B B', param1, param2))

    @staticmethod
    def req_firmware_version(): return RosmasterProtocol.request_data(0x51)
    @staticmethod
    def req_motion_data(): return RosmasterProtocol.request_data(0x0A)
    @staticmethod
    def req_imu_data(): return RosmasterProtocol.request_data(0x0B)
    @staticmethod
    def req_attitude_angle(): return RosmasterProtocol.request_data(0x0C)
    @staticmethod
    def req_bus_servo_position(servo_id): return RosmasterProtocol.request_data(0x20, servo_id)
    @staticmethod
    def req_arm_all_joints(): return RosmasterProtocol.request_data(0x23, 0x01)
    @staticmethod
    def req_pid(): return RosmasterProtocol.request_data(0x13, 0x01)
    @staticmethod
    def req_yaw_pid(): return RosmasterProtocol.request_data(0x14, 0x05)

    # ==========================================
    # 接收数据解析核心 (Board -> Host)
    # ==========================================
    @staticmethod
    def _parse_payload(func_code, data):
        """根据功能字解包 Payload 返回标准字典"""
        try:
            if func_code == 0x51:  # 固件版本号
                major, minor = struct.unpack('<B B', data)
                return {"type": "firmware_version", "major": major, "minor": minor}

            elif func_code == 0x0A:  # 运动数据 (里程计和电压)
                vx, vy, vz, vbat = struct.unpack('<h h h B', data)
                return {
                    "type": "motion_data",
                    "linear_x": vx / 1000.0,
                    "linear_y": vy / 1000.0,
                    "angular_z": vz / 1000.0,
                    "voltage": vbat / 10.0
                }

            elif func_code == 0x0B:  # IMU原始数据
                # 由于CSV长度标识可能有差异，这里通过动态长度支持9轴或6轴
                vals = struct.unpack(f'<{len(data)//2}h', data)
                res = {"type": "imu_raw", "gyro": vals[0:3], "accel": vals[3:6]}
                if len(vals) >= 9:
                    res["mag"] = vals[6:9]
                return res

            elif func_code == 0x0C:  
                # 注意：CSV文档中有重叠和笔误，0x0C通常为姿态角，但也可能混用了机械臂解析
                if len(data) == 6:  # 3个短整型 -> 姿态角
                    yaw, roll, pitch = struct.unpack('<h h h', data)
                    return {
                        "type": "attitude",
                        "yaw": yaw / 100.0,    # 根据一般惯例除以系数，具体需视实际底层缩放
                        "roll": roll / 100.0,
                        "pitch": pitch / 100.0
                    }
                elif len(data) == 12:  # 6个短整型 -> 机械臂返回
                    joints = struct.unpack('<H H H H H H', data)
                    return {"type": "arm_joints", "joints": joints}

            elif func_code == 0x24:  # 校准结果
                servo_id, state = struct.unpack('<B B', data)
                return {"type": "calibration_result", "id": servo_id, "state": state}

            elif func_code == 0x20:  # 总线舵机位置
                servo_id, position = struct.unpack('<B H', data)
                return {"type": "bus_servo_pos", "id": servo_id, "position": position}

            elif func_code == 0x13:  # 速度PID
                valid, p, i, d = struct.unpack('<B h h h', data)
                return {"type": "velocity_pid", "p": p/1000.0, "i": i/1000.0, "d": d/1000.0}

            elif func_code == 0x14:  # 偏航角PID
                valid, p, i, d = struct.unpack('<B h h h', data)
                return {"type": "yaw_pid", "p": p/1000.0, "i": i/1000.0, "d": d/1000.0}

            else:
                return {"type": "unknown", "func_code": hex(func_code), "raw_data": data.hex()}
                
        except struct.error as e:
            return {"error": f"Struct unpack error: {str(e)}", "func_code": hex(func_code)}
    def req_yaw_pid(): return RosmasterProtocol.request_data(0x14, 0x05)

    # ==========================================
    # 接收数据解析核心 (Board -> Host)
    # ==========================================
    @staticmethod
    def _parse_payload(func_code, data):
        """根据功能字解包 Payload 返回标准字典"""
        try:
            if func_code == 0x51:  # 固件版本号
                major, minor = struct.unpack('<B B', data)
                return {"type": "firmware_version", "major": major, "minor": minor}

            elif func_code == 0x0A:  # 运动数据 (里程计和电压)
                vx, vy, vz, vbat = struct.unpack('<h h h B', data)
                return {
                    "type": "motion_data",
                    "linear_x": vx / 1000.0,
                    "linear_y": vy / 1000.0,
                    "angular_z": vz / 1000.0,
                    "voltage": vbat / 10.0
                }

            elif func_code == 0x0B:  # IMU原始数据
                # 由于CSV长度标识可能有差异，这里通过动态长度支持9轴或6轴
                vals = struct.unpack(f'<{len(data)//2}h', data)
                res = {"type": "imu_raw", "gyro": vals[0:3], "accel": vals[3:6]}
                if len(vals) >= 9:
                    res["mag"] = vals[6:9]
                return res

            elif func_code == 0x0C:  
                # 注意：CSV文档中有重叠和笔误，0x0C通常为姿态角，但也可能混用了机械臂解析
                if len(data) == 6:  # 3个短整型 -> 姿态角
                    yaw, roll, pitch = struct.unpack('<h h h', data)
                    return {
                        "type": "attitude",
                        "yaw": yaw / 100.0,    # 根据一般惯例除以系数，具体需视实际底层缩放
                        "roll": roll / 100.0,
                        "pitch": pitch / 100.0
                    }
                elif len(data) == 12:  # 6个短整型 -> 机械臂返回
                    joints = struct.unpack('<H H H H H H', data)
                    return {"type": "arm_joints", "joints": joints}

            elif func_code == 0x24:  # 校准结果
                servo_id, state = struct.unpack('<B B', data)
                return {"type": "calibration_result", "id": servo_id, "state": state}

            elif func_code == 0x20:  # 总线舵机位置
                servo_id, position = struct.unpack('<B H', data)
                return {"type": "bus_servo_pos", "id": servo_id, "position": position}

            elif func_code == 0x13:  # 速度PID
                valid, p, i, d = struct.unpack('<B h h h', data)
                return {"type": "velocity_pid", "p": p/1000.0, "i": i/1000.0, "d": d/1000.0}

            elif func_code == 0x14:  # 偏航角PID
                valid, p, i, d = struct.unpack('<B h h h', data)
                return {"type": "yaw_pid", "p": p/1000.0, "i": i/1000.0, "d": d/1000.0}

            else:
                return {"type": "unknown", "func_code": hex(func_code), "raw_data": data.hex()}
                
        except struct.error as e:
            return {"error": f"Struct unpack error: {str(e)}", "func_code": hex(func_code)}
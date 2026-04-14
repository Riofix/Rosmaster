# -*- coding: utf-8 -*-
import struct

"""
底盘与起重臂通讯协议全集字典
本文件将 C 语言体系下的 `app_protocol.h` 和 `protocol.h` 所定义的全部十六进制指令，
一一映射为 Python 中可直接调用的接口。
"""

class CraneCodec:
    """吊具从机 (ESP8266) 协议库: 包头仅为单字节 0xFF / 0xFB"""
    
    @staticmethod
    def calc_checksum(frame_bytes):
        return sum(frame_bytes[:-1]) % 256

    @staticmethod
    def pack(func_code, data_bytes=b''):
        frame = bytearray((0xFF, func_code, len(data_bytes)))
        frame.extend(data_bytes)
        frame.append(CraneCodec.calc_checksum(frame))
        return bytes(frame)

    # ================= 发送 (Pack) 接口 =================
    @staticmethod
    def pack_en_control(addr, en):
        return CraneCodec.pack(0x50, struct.pack('<B B', addr, 1 if en else 0))

    @staticmethod
    def pack_vel_control(addr, dir_byte, vel, acc):
        return CraneCodec.pack(0x51, struct.pack('<B B H B', addr, dir_byte, vel, acc))

    @staticmethod
    def pack_emm_pos_ctrl(addr, dir_byte, vel, acc, target_ticks, is_rel):
        data = struct.pack('<B B H B I B', addr, dir_byte, vel, acc, target_ticks, 1 if is_rel else 0)
        return CraneCodec.pack(0x52, data)

    @staticmethod
    def pack_stop_now(addr):
        return CraneCodec.pack(0x53, struct.pack('<B', addr))

    @staticmethod
    def pack_origin_trigger(addr, dir_byte):
        return CraneCodec.pack(0x57, struct.pack('<B B', addr, dir_byte))

    @staticmethod
    def pack_outlet_servo(ch, angle):
        return CraneCodec.pack(0x70, struct.pack('<B B', ch, angle))

    @staticmethod
    def pack_vacuum_duty(duty):
        duty = max(0, min(100, duty))
        return CraneCodec.pack(0x72, struct.pack('<B', duty))
        
    @staticmethod
    def pack_tracker_set_goal(target_mm):
        return CraneCodec.pack(0x7A, struct.pack('<f', target_mm))

    # ================= 接收 (Parse) 接口 =================
    @staticmethod
    def parse_crane_rx(func_code, data):
        """解析 ESP8266 发上来的状态与上报数据"""
        if func_code == 0x60: # CMD_TX_ACK_PARAM
            return {'type': 'ack_param', 'data': list(data)}
        elif func_code == 0x61: # CMD_TX_ACK_OK
            return {'type': 'ack_ok', 'motor_req': data[0] if len(data)>0 else 0}
        elif func_code == 0x62: # CMD_TX_ACK_ERR
            return {'type': 'ack_err', 'motor_req': data[0] if len(data)>0 else 0, 'err_code': data[1] if len(data)>1 else 0}
        elif func_code == 0x63: # CMD_TX_REPORT_POS
            return {'type': 'report_pos', 'motor_addr': data[0] if len(data)>0 else 0}
        elif func_code == 0x64: # CMD_TX_MPU_DATA (Roll, Pitch, Yaw)
            if len(data) >= 12:
                roll, pitch, yaw = struct.unpack('<f f f', data[0:12])
                return {'type': 'mpu_carriage', 'roll': roll, 'pitch': pitch, 'yaw': yaw}
        elif func_code == 0x65: # CMD_TX_TRACKER_DATA
            if len(data) >= 9:
                pos, target = struct.unpack('<f f', data[0:8])
                mode = data[8]
                raw_ticks = struct.unpack('<i', data[9:13])[0] if len(data)>=13 else 0
                return {'type': 'emm_odom', 'pos': pos, 'target': target, 'mode': mode, 'raw_ticks': raw_ticks}
        return {'type': 'unknown', 'func_code': func_code}


class ChassisCodec:
    """Rosmaster 主底盘协议库: [0xFF, 0xFC/0xFB, LEN, CMD, Payload..., CHK]"""
    
    @staticmethod
    def calc_checksum(frame_bytes):
        return sum(frame_bytes[2:-1]) % 256

    @staticmethod
    def pack(func_code, data_bytes=b''):
        length = len(data_bytes) + 2
        frame = bytearray((0xFF, 0xFC, length, func_code))
        frame.extend(data_bytes)
        frame.append(ChassisCodec.calc_checksum(frame))
        return bytes(frame)

    # ================= 发送 (Pack) 接口 =================
    @staticmethod
    def pack_velocity(vx, vy, vth, adjust=0):
        # bit7 if adjust=1
        parm = 0x80 if adjust else 0x00
        return ChassisCodec.pack(0x12, struct.pack('<B h h h', parm, int(vx*1000), int(vy*1000), int(vth*1000)))

    @staticmethod
    def pack_beep(time_ms):
        return ChassisCodec.pack(0x02, struct.pack('<H', time_ms))

    @staticmethod
    def pack_rgb(index, r, g, b):
        return ChassisCodec.pack(0x05, struct.pack('<B B B B', index, r, g, b))

    @staticmethod
    def pack_pwm_servo(servo_id, angle):
        return ChassisCodec.pack(0x03, struct.pack('<B B', servo_id, angle))

    @staticmethod
    def pack_reset_state():
        return ChassisCodec.pack(0x0F, b'\x5F')

    # ================= 接收 (Parse) 接口 =================
    @staticmethod
    def parse_chassis_rx(func_code, data):
        """解析底盘主动上报的所有数据包 (速度、MPU、编码器等)"""
        if func_code == 0x0A: # FUNC_REPORT_SPEED
            if len(data) >= 7:
                vx, vy, vz, vbat = struct.unpack('<h h h B', data[:7])
                return {'type': 'motion', 'vx': vx/1000.0, 'vy': vy/1000.0, 'vth': vz/1000.0, 'vbat': vbat/10.0}
        elif func_code == 0x0B: # FUNC_REPORT_MPU_RAW
            if len(data) >= 12:
                ax, ay, az, gx, gy, gz = struct.unpack('<h h h h h h', data[:12])
                return {'type': 'mpu_raw', 'ax': ax, 'ay': ay, 'az': az, 'gx': gx, 'gy': gy, 'gz': gz}
        elif func_code == 0x0C: # FUNC_REPORT_IMU_ATT
            if len(data) >= 6:
                roll, pitch, yaw = struct.unpack('<h h h', data[:6])
                return {'type': 'imu_att', 'roll': roll/100.0, 'pitch': pitch/100.0, 'yaw': yaw/100.0}
        elif func_code == 0x0D: # FUNC_REPORT_ENCODER
            if len(data) >= 16:
                m1, m2, m3, m4 = struct.unpack('<i i i i', data[:16])
                return {'type': 'encoder', 'm1': m1, 'm2': m2, 'm3': m3, 'm4': m4}
        return {'type': 'unknown', 'func_code': func_code}

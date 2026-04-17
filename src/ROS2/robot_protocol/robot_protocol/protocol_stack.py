# -*- coding: utf-8 -*-
import struct


class CraneCodec:
    RX_HEADERS = (0xFF, 0xFB)

    @staticmethod
    def calc_checksum(frame_bytes):
        return sum(frame_bytes[:-1]) % 256

    @staticmethod
    def pack(func_code, data_bytes=b''):
        frame = bytearray((0xFF, func_code, len(data_bytes)))
        frame.extend(data_bytes)
        frame.append(CraneCodec.calc_checksum(frame))
        return bytes(frame)

    @staticmethod
    def pack_emm_pos_ctrl(addr, dir_byte, vel, acc, target_ticks, is_rel):
        data = struct.pack('<B B H B I B', addr, dir_byte, vel, acc, target_ticks, 1 if is_rel else 0)
        return CraneCodec.pack(0x52, data)

    @staticmethod
    def pack_outlet_servo(ch, angle):
        return CraneCodec.pack(0x70, struct.pack('<B B', ch, angle))

    @staticmethod
    def pack_vacuum_duty(duty):
        duty = max(0, min(100, int(duty)))
        return CraneCodec.pack(0x72, struct.pack('<B', duty))

    @staticmethod
    def pack_tracker_set_goal(target_mm):
        return CraneCodec.pack(0x7A, struct.pack('<f', float(target_mm)))

    @staticmethod
    def parse_rx(func_code, data):
        if func_code == 0x60:
            return {'type': 'ack_param', 'data': list(data)}
        if func_code == 0x61:
            return {'type': 'ack_ok', 'motor_req': data[0] if len(data) > 0 else 0}
        if func_code == 0x62:
            return {
                'type': 'ack_err',
                'motor_req': data[0] if len(data) > 0 else 0,
                'err_code': data[1] if len(data) > 1 else 0,
            }
        if func_code == 0x63:
            return {'type': 'report_pos', 'motor_addr': data[0] if len(data) > 0 else 0}
        if func_code == 0x64 and len(data) >= 12:
            roll, pitch, yaw = struct.unpack('<f f f', data[0:12])
            return {'type': 'mpu_carriage', 'roll': roll, 'pitch': pitch, 'yaw': yaw}
        if func_code == 0x65 and len(data) >= 9:
            pos, target = struct.unpack('<f f', data[0:8])
            mode = data[8]
            raw_ticks = struct.unpack('<i', data[9:13])[0] if len(data) >= 13 else 0
            return {'type': 'emm_odom', 'pos': pos, 'target': target, 'mode': mode, 'raw_ticks': raw_ticks}
        return {'type': 'unknown', 'func_code': func_code}

    @staticmethod
    def parse_tx(func_code, data):
        if func_code == 0x52 and len(data) >= 10:
            addr, dir_byte, vel, acc, target_ticks, is_rel = struct.unpack('<B B H B I B', data[:10])
            return {
                'type': 'emm_pos_ctrl',
                'addr': addr,
                'dir': dir_byte,
                'vel': vel,
                'acc': acc,
                'target_ticks': target_ticks,
                'is_rel': bool(is_rel),
            }
        if func_code == 0x70 and len(data) >= 2:
            channel, angle = struct.unpack('<B B', data[:2])
            return {'type': 'outlet_servo', 'channel': channel, 'angle': angle}
        if func_code == 0x72 and len(data) >= 1:
            return {'type': 'vacuum', 'duty': data[0]}
        if func_code == 0x7A and len(data) >= 4:
            (target_mm,) = struct.unpack('<f', data[:4])
            return {'type': 'tracker_goal', 'target_mm': target_mm}
        return {'type': 'unknown', 'func_code': func_code}


class ChassisCodec:
    RX_DEVICE_IDS = (0xFC, 0xFB)

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

    @staticmethod
    def pack_velocity(vx, vy, vth, adjust=0):
        parm = 0x80 if adjust else 0x00
        return ChassisCodec.pack(
            0x12,
            struct.pack('<B h h h', parm, int(vx * 1000), int(vy * 1000), int(vth * 1000)),
        )

    @staticmethod
    def pack_beep(time_ms):
        return ChassisCodec.pack(0x02, struct.pack('<H', int(time_ms)))

    @staticmethod
    def parse_rx(func_code, data):
        if func_code == 0x0A and len(data) >= 7:
            vx, vy, vz, vbat = struct.unpack('<h h h B', data[:7])
            return {'type': 'motion', 'vx': vx / 1000.0, 'vy': vy / 1000.0, 'vth': vz / 1000.0, 'vbat': vbat / 10.0}
        if func_code == 0x0B and len(data) >= 12:
            ax, ay, az, gx, gy, gz = struct.unpack('<h h h h h h', data[:12])
            return {'type': 'mpu_raw', 'ax': ax, 'ay': ay, 'az': az, 'gx': gx, 'gy': gy, 'gz': gz}
        if func_code == 0x0C and len(data) >= 6:
            roll, pitch, yaw = struct.unpack('<h h h', data[:6])
            return {'type': 'imu_att', 'roll': roll / 100.0, 'pitch': pitch / 100.0, 'yaw': yaw / 100.0}
        if func_code == 0x0D and len(data) >= 16:
            m1, m2, m3, m4 = struct.unpack('<i i i i', data[:16])
            return {'type': 'encoder', 'm1': m1, 'm2': m2, 'm3': m3, 'm4': m4}
        return {'type': 'unknown', 'func_code': func_code}


class CraneStreamParser:
    def __init__(self):
        self._buffer = bytearray()

    def feed(self, chunk):
        if not chunk:
            return []
        self._buffer.extend(chunk)
        frames = []
        while True:
            start = next((i for i, b in enumerate(self._buffer) if b in CraneCodec.RX_HEADERS), -1)
            if start < 0:
                self._buffer.clear()
                break
            if start > 0:
                del self._buffer[:start]
            if len(self._buffer) < 4:
                break
            data_len = self._buffer[2]
            frame_len = data_len + 4
            if len(self._buffer) < frame_len:
                break
            frame = bytes(self._buffer[:frame_len])
            del self._buffer[:frame_len]
            if frame[-1] != CraneCodec.calc_checksum(frame):
                continue
            frames.append((frame[0], frame[1], frame[3:-1]))
        return frames


class ChassisStreamParser:
    def __init__(self):
        self._buffer = bytearray()

    def feed(self, chunk):
        if not chunk:
            return []
        self._buffer.extend(chunk)
        frames = []
        while True:
            start = next(
                (
                    i for i in range(len(self._buffer) - 1)
                    if self._buffer[i] == 0xFF and self._buffer[i + 1] in ChassisCodec.RX_DEVICE_IDS
                ),
                -1,
            )
            if start < 0:
                if self._buffer and self._buffer[-1] == 0xFF:
                    self._buffer[:] = self._buffer[-1:]
                else:
                    self._buffer.clear()
                break
            if start > 0:
                del self._buffer[:start]
            if len(self._buffer) < 5:
                break
            length = self._buffer[2]
            if length < 2:
                del self._buffer[0]
                continue
            frame_len = length + 3
            if len(self._buffer) < frame_len:
                break
            frame = bytes(self._buffer[:frame_len])
            del self._buffer[:frame_len]
            if frame[-1] != ChassisCodec.calc_checksum(frame):
                continue
            frames.append((frame[1], frame[3], frame[4:-1]))
        return frames

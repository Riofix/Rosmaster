#!/usr/bin/env python3
"""
语音模块模拟器 — 用于在没有真实 SU-03T 硬件时测试 voice_node。

用法:
  # 方式1: 用虚拟串口对 (socat)
  #   socat -d -d pty,raw,echo=0 pty,raw,echo=0
  #   输出: /dev/pts/3 和 /dev/pts/4
  # 终端1: python3 voice_sim.py /dev/pts/4
  # 终端2: ros2 run robot_voice voice_node --ros-args -p port:=/dev/pts/3

  # 方式2: 直接用键盘模拟 (不需要串口)
  #   python3 voice_sim.py --stdin
"""

import sys
import time
import serial
import threading
import select
import json

# ── AA 55 协议帧 ──────────────────────────────────────────────
FRAMES = {
    # 用户语音命令词 (模拟模块→主机)
    '1':  bytes([0xAA, 0x55, 0x01, 0x00, 0xFB]),  # 欢迎语
    '2':  bytes([0xAA, 0x55, 0x02, 0x6F, 0xFB]),  # 休息语
    '3':  bytes([0xAA, 0x55, 0x03, 0x00, 0xFB]),  # 你好地瓜
    '20': bytes([0xAA, 0x55, 0xFF, 0x01, 0xFB]),  # 地瓜启动 ★
    '21': bytes([0xAA, 0x55, 0xFF, 0x02, 0xFB]),  # 地瓜初始化
    # 主机下发后模块返回确认
    's20': bytes([0xAA, 0x55, 0xFF, 0x01, 0xFB]),  # 应答: 地瓜启动
    's21': bytes([0xAA, 0x55, 0xFF, 0x02, 0xFB]),  # 应答: 地瓜初始化
}


def serial_mode(port):
    """串口模式: 连接真实/虚拟串口"""
    ser = serial.Serial(port, 115200, timeout=0.1)
    print(f'[SIM] 串口 {port} 已打开, 115200')
    print('[SIM] 按键发送语音指令:')
    print('  1  = 欢迎语 (id=1)')
    print('  2  = 休息语 (id=2)')
    print('  3  = 你好地瓜 (id=3)')
    print('  20 = 地瓜启动 (id=20) ★')
    print('  21 = 地瓜初始化 (id=21)')
    print('  q  = 退出')
    print('[SIM] 同时监听串口接收 (主机下发播报指令):')

    def rx_thread():
        while True:
            try:
                if ser.in_waiting:
                    data = ser.read(ser.in_waiting)
                    print(f'\n[SIM RX] ← 主机下发: {data.hex(" ").upper()}')
            except:
                break

    t = threading.Thread(target=rx_thread, daemon=True)
    t.start()

    try:
        while True:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.readline().strip()
                if key == 'q':
                    break
                if key in FRAMES:
                    frame = FRAMES[key]
                    ser.write(frame)
                    print(f'[SIM TX] → 发送: {frame.hex(" ").upper()}  (id={key})')
                else:
                    print(f'[SIM] 未知按键: {key}')
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        print('[SIM] 退出')


def stdin_mode():
    """纯键盘模式: 不需要串口, 直接打印主机下发的指令"""
    print('[SIM] 键盘模拟模式 (无需串口)')
    print('[SIM] 输入 JSON 模拟主机下发播报指令:')
    print('  例: {"cmd_id": 21}   → 模拟主机要求播报"地瓜初始化"')
    print('  例: {"cmd_id": 20}   → 模拟主机要求播报"地瓜启动"')
    print('  例: {"cmd_id": 1}    → 模拟主机要求播报"欢迎语"')
    print()
    print('[SIM] 输入数字模拟语音模块上报命令词:')
    print('  1, 2, 3, 20, 21')

    for _id, frame in FRAMES.items():
        if not _id.startswith('s'):
            print(f'    {_id}:  {frame.hex(" ").upper()}')

    print()

    try:
        while True:
            line = sys.stdin.readline().strip()
            if not line:
                continue
            if line == 'q':
                break

            # 尝试 JSON 解析
            if line.startswith('{'):
                try:
                    data = json.loads(line)
                    cmd_id = data.get('cmd_id', 0)
                    frame = FRAMES.get(f's{cmd_id}', None) or FRAMES.get(str(cmd_id))
                    if frame:
                        print(f'[SIM TX] → 主机下发播报: {frame.hex(" ").upper()}  (cmd_id={cmd_id})')
                    else:
                        print(f'[SIM] 未知 cmd_id: {cmd_id}')
                except json.JSONDecodeError:
                    print(f'[SIM] 无效 JSON: {line}')
            else:
                # 数字 = 模拟语音模块上报
                frame = FRAMES.get(line)
                if frame:
                    print(f'[SIM RX] ← 语音模块上报: {frame.hex(" ").upper()}  (id={line})')
                else:
                    print(f'[SIM] 未知命令: {line}')
    except KeyboardInterrupt:
        pass
    print('[SIM] 退出')


if __name__ == '__main__':
    if '--stdin' in sys.argv:
        stdin_mode()
    elif len(sys.argv) > 1:
        serial_mode(sys.argv[1])
    else:
        print('用法:')
        print('  python3 voice_sim.py /dev/pts/N    # 串口模式 (真实/虚拟)')
        print('  python3 voice_sim.py --stdin       # 键盘模拟模式')

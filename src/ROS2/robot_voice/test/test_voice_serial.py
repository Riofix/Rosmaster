#!/usr/bin/env python3
"""
语音模块串口硬件连通性测试 — 不依赖 ROS2, 直接读写串口。

用法:
  python3 test_voice_serial.py /dev/broadcast
"""

import sys
import serial
import time
import threading

AA55_FRAMES = {
    'init_ok':      bytes([0xAA, 0x55, 0xFF, 0x02, 0xFB]),  # 地瓜初始化 → "好的，已初始化完成"
    'start_confirm': bytes([0xAA, 0x55, 0xFF, 0x01, 0xFB]),  # 地瓜启动确认
    'welcome':       bytes([0xAA, 0x55, 0x01, 0x00, 0xFB]),  # 欢迎语
    'hello':         bytes([0xAA, 0x55, 0x03, 0x00, 0xFB]),  # 你好地瓜
    'vol_up':        bytes([0xAA, 0x55, 0x04, 0x00, 0xFB]),  # 增大音量
    'vol_down':      bytes([0xAA, 0x55, 0x05, 0x00, 0xFB]),  # 减小音量
}


def main():
    if len(sys.argv) < 2:
        print(f'用法: python3 {sys.argv[0]} <串口设备>')
        print(f'示例: python3 {sys.argv[0]} /dev/broadcast')
        sys.exit(1)

    port = sys.argv[1]

    # ── 1. 打开串口 ────────────────────────────────────────
    print(f'[1/4] 打开串口 {port} ...')
    try:
        ser = serial.Serial(port, 115200,
                            bytesize=serial.EIGHTBITS,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            timeout=1.0)
        print(f'      ✓ 串口已打开: {ser.name}')
    except Exception as e:
        print(f'      ✗ 失败: {e}')
        print(f'      请检查:')
        print(f'        1. 语音模块是否已上电')
        print(f'        2. USB 线是否插好 (lsusb | grep 1a86)')
        print(f'        3. udev 规则是否生效 (ls -la /dev/broadcast)')
        sys.exit(1)

    # ── 启动接收线程 ──────────────────────────────────────
    rx_running = True

    def rx_loop():
        while rx_running:
            try:
                if ser.in_waiting > 0:
                    data = ser.read(ser.in_waiting)
                    print(f'\n      ← 收到: {data.hex(" ").upper()}')
                    if data[:2] == b'\xaa\x55':
                        print(f'      ✓ 有效 AA 55 帧!')
            except:
                break

    rx_thread = threading.Thread(target=rx_loop, daemon=True)
    rx_thread.start()
    print(f'      ✓ 接收线程已启动')

    # ── 2. 发送测试帧 ──────────────────────────────────────
    print(f'\n[2/4] 发送测试指令...')

    test_sequence = [
        ('vol_up',    '增大音量 (扬声器应有反馈)'),
        ('vol_down',  '减小音量'),
        ('init_ok',   '初始化完成播报 → "好的，已初始化完成"'),
    ]

    for name, desc in test_sequence:
        frame = AA55_FRAMES[name]
        print(f'\n      → 发送: {frame.hex(" ").upper()}  ({name})')
        print(f'        说明: {desc}')
        ser.write(frame)
        ser.flush()
        time.sleep(2.0)  # 等播报完成

    # ── 3. 监听 (对着模块说话) ────────────────────────────
    print(f'\n[3/4] 监听模式 — 对着语音模块说话 (10秒)...')
    print(f'      试试说: "你好地瓜", "地瓜启动", "增大音量"')
    print(f'      按 Ctrl+C 跳过...')
    try:
        time.sleep(10)
    except KeyboardInterrupt:
        pass

    # ── 4. 结果汇总 ────────────────────────────────────────
    print(f'\n[4/4] 测试结果汇总:')
    print(f'      串口通信: ✓ 正常')
    print(f'      下行(主机→模块): ✓ 已测试')
    print(f'      上行(模块→主机): 请根据上面收到的帧判断:')
    print(f'        - 看到 AA 55 开头 → 模块上行正常')
    print(f'        - 没看到 → 检查模块麦克风/唤醒词')
    print(f'        - 收到乱码 → 波特率或接线问题')

    rx_running = False
    ser.close()
    print(f'\n测试结束。')


if __name__ == '__main__':
    main()

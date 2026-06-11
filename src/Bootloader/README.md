# Bootloader 方案记录

## 当前方案：双槽交替

```
0x08000000 ── Bootloader (7KB)
0x08001C00 ── slot_state (1KB)
0x08002000 ── Slot A (28KB)
0x08009000 ── Slot B (28KB)
```

### 流程
1. APP 收到 OTA 指令 → `slot_set_enter_update()` → `NVIC_SystemReset()`
2. Bootloader 启动 → 读 `enter_update=1` → 进入更新模式
3. 擦除**非活跃槽位** → 发 `READY TO RECEIVE`
4. PC 流式发送 bin → DMA 接收 → 写入非活跃槽位
5. 3 秒静默 → 校验 CRC
6. CRC OK → `update_pending=1` → 复位 → 切换 `active_slot` → 跳转新 APP
7. CRC FAIL → 发 `CRC FAIL` → 重试(最多3次) → 回退旧 APP

### 编译
- Slot A: IROM1 = `0x08002000`
- Slot B: IROM1 = `0x08009000`
- 每次编译需要手动切 Target（或改 IROM1 Start）
- APP 代码开头 `SCB->VTOR = (uint32_t)&__Vectors;` 自动适配

### 优点
- OTA 失败不损坏运行中的 APP
- 可随时回滚旧版本

### 缺点
- 编译两份固件，每次需要切换 IROM1
- 需要记住当前在哪个槽位


## 备选方案：固定 APP 位

```
0x08000000 ── Bootloader (7KB)
0x08002000 ── APP 运行区 (28KB)
0x08009000 ── 固件暂存区 (28KB)
```

### 流程
1. Bootloader 接收新固件 → 写入**暂存区**
2. CRC 校验通过 → 将暂存区内容复制到 APP 运行区
3. CRC 校验失败 → 暂存区不管，继续跑旧 APP
4. 跳转 APP

### 编译
- 只编译一次：IROM1 = `0x08002000`
- 不用切 Target

### 优点
- 只编译一份固件
- 不用管槽位

### 缺点
- Flash 复制期间断电 → **变砖**
- 复制 28KB 耗时约 1~2 秒
- 不能保留旧版本做回滚

## PC 端通信协议

| 时机 | STM32 → PC |
|:---|:---|
| 进入 OTA 模式 | `BOOT` |
| 擦除完成 | `ERASE OK` + `READY TO RECEIVE` |
| 镜像头错误 | `BAD HEADER` |
| CRC 校验失败 | `CRC FAIL` |
| 重试 | `RETRY N/3` |
| 30 秒无数据 | `TIMEOUT NO DATA` |
| 校验通过 | `CRC OK` |
| 更新成功 | `UPDATE SUCCESS` |

## 文件清单

```
src/
├── Common/
│   └── slot_manager.h          # 共享头文件 (分区/结构体)
├── Bootloader/                  # Bootloader 工程
│   ├── CMSIS/
│   ├── FWlib/                   # 仅保留 flash/gpio/rcc/dma/usart/misc
│   ├── Src/
│   │   ├── APP/
│   │   │   ├── main.c           # 主流程
│   │   │   ├── flash_if.c/.h    # Flash 操作
│   │   │   ├── ymodem.c/.h      # 预留 (当前未使用)
│   │   │   ├── slot_manager.c   # 槽位管理
│   │   │   └── stm32f10x_it.c/.h
│   │   └── BSP/
│   │       ├── bsp_usart.c/.h   # USART2 DMA
│   │       ├── bsp_systick.c/.h # 延时
│   │       ├── OLED.c/.h        # 显示
│   │       └── bsp_esp8266.c/.h # WiFi (保留)
│   └── Objects/
└── Stm32F103C8T6/               # APP 工程
    └── Src/
        ├── APP/main.c           # 加 SCB->VTOR = &__Vectors
        └── Protocol/cmd_handle.c # 加 Handle_OTA_Start
```

## OLED 显示设计

```
启动:       BOOTLOADER V1.0
            SLOT [A/B] ACTIVE

OTA 模式:   OTA MODE

擦除:       ERASING SLOT...

接收:       RECEIVING...
            12345 / 24576 B
            50%

校验:       VERIFYING...
            CRC OK

失败:       CRC FAIL
            BAD HEADER

重试:       RETRY 2/3

超时:       TIMEOUT

成功:       OK! RESET...

跳转:       JUMP TO APP
```

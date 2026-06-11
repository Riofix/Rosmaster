/**
 ******************************************************************************
 * @file    slot_manager.h
 * @brief   Bootloader 与 APP 共享的头文件
 *          定义 Flash 分区地址、槽位状态结构体、镜像头结构体
 *
 *          分区布局:
 *          0x08000000 ┌──────────────┐
 *                     │  Bootloader   │  7KB (页 0~6)
 *          0x08001C00 ├──────────────┤
 *                     │  slot_state   │  1KB (页 7, 独占)
 *          0x08002000 ├──────────────┤
 *                     │  APP Slot A   │  28KB (页 8~35)
 *          0x08009000 ├──────────────┤
 *                     │  APP Slot B   │  28KB (页 36~63)
 *          0x08010000 └──────────────┘
 ******************************************************************************
 */

#ifndef __SLOT_MANAGER_H
#define __SLOT_MANAGER_H

#include <stdint.h>

/* ========================== Flash 分区定义 ========================== */

#define BL_BASE                 0x08000000u
#define BL_SIZE                 0x00001C00u   /* Bootloader 代码: 7KB */

#define SLOT_STATE_ADDR         0x08001C00u   /* 槽位状态页基址: 1KB */
#define SLOT_STATE_PAGE_SIZE    0x00000400u   /* 1 个 Flash 页 = 1KB */

#define SLOT_A_BASE             0x08002000u   /* APP Slot A 基址 */
#define SLOT_A_SIZE             0x00007000u   /* 28KB */

#define SLOT_B_BASE             0x08009000u   /* APP Slot B 基址 */
#define SLOT_B_SIZE             0x00007000u   /* 28KB */

/* ========================== 槽位枚举 ========================== */

typedef enum {
    SLOT_A = 0,
    SLOT_B = 1
} slot_t;

/* 根据 slot 获取基址 */
#define SLOT_BASE(s)  ((s) == SLOT_A ? SLOT_A_BASE : SLOT_B_BASE)

/* ========================== 槽位状态结构体 ========================== */
/*
 * 存放在 SLOT_STATE_ADDR (0x08001C00)，独占一页 Flash。
 * 注意: sizeof(slot_state_t) 远小于 1KB, 其余空间保留。
 */

#define SLOT_STATE_MAGIC        0xAA55A5A5u   /* 结构体有效标志 */

typedef struct {
    uint32_t magic;            /* 魔数, 必须 = SLOT_STATE_MAGIC */
    uint8_t  active_slot;      /* 当前活跃槽位: 0=SlotA, 1=SlotB */
    uint8_t  enter_update;     /* 1=APP 请求进入更新模式 */
    uint8_t  update_pending;   /* 1=新固件已写入, 待激活 */
    uint8_t  slot_a_valid;     /* Slot A 固件有效标志 */
    uint8_t  slot_b_valid;     /* Slot B 固件有效标志 */
    uint32_t crc;              /* 本结构体前 12 字节的 CRC32 校验 */
} slot_state_t;

/* ========================== 镜像头结构体 ========================== */
/*
 * 存放在每个 APP Slot 的固定偏移处 (SlotBase + 0x100)。
 * Bootloader 在跳转前校验此头部。
 */

#define IMAGE_HEADER_OFFSET     0x100u        /* 镜像头在槽位内的偏移 */
#define IMAGE_HEADER_MAGIC      0xAABBCCDDu   /* 镜像有效标志 */

typedef struct {
    uint32_t magic;            /* 魔数, 必须 = IMAGE_HEADER_MAGIC */
    uint32_t version;          /* 固件版本, 如 0x00010002 = v1.0.2 */
    uint32_t image_size;       /* 固件有效载荷大小 (字节) */
    uint32_t image_crc;        /* 整包固件 CRC32 校验 */
} image_header_t;

/* ========================== 辅助宏 ========================== */

/* 获取指定槽位的镜像头地址 */
#define IMAGE_HEADER_ADDR(slot)  (SLOT_BASE(slot) + IMAGE_HEADER_OFFSET)

/* ========================== 槽位管理函数 (Bootloader 实现) ========================== */
#ifdef __cplusplus
extern "C" {
#endif

slot_state_t slot_get_state(void);
void         slot_set_state(const slot_state_t *state);
slot_t       slot_get_active(void);
slot_t       slot_get_inactive(void);
uint32_t     slot_get_inactive_addr(void);
void         slot_set_enter_update(void);
void         slot_clear_enter_update(void);
void         slot_set_update_pending(void);
void         slot_switch_active(void);

#ifdef __cplusplus
}
#endif

#endif /* __SLOT_MANAGER_H */

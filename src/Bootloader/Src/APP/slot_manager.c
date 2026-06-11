/**
 * @file slot_manager.c
 * @brief 槽位状态管理 — 读写 FLASH 中的 slot_state_t
 */

#include "slot_manager.h"
#include "stm32f10x_flash.h"
#include <string.h>

/* ========== 内部 CRC 计算 ========== */
/*
 * 对 slot_state_t 前 12 字节计算 CRC32，
 * 因为 uint8_t 数组保证了按字节计算，不涉及对齐问题。
 */
static uint32_t crc32_slot(const uint8_t *data, uint32_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320u;
            else
                crc >>= 1;
        }
    }
    return ~crc;
}

static uint32_t calc_slot_state_crc(const slot_state_t *s) {
    /* magic(4) + active(1) + enter(1) + pending(1) + a_valid(1) + b_valid(1) = 9 bytes */
    return crc32_slot((const uint8_t *)s, 9);
}

/* ========== 读操作 ========== */
/*
 * FLASH 是内存映射的，直接指针读取。
 * 返回前校验 magic + CRC。
 */
slot_state_t slot_get_state(void) {
    const slot_state_t *p = (const slot_state_t *)SLOT_STATE_ADDR;
    slot_state_t state;

    if (p->magic != SLOT_STATE_MAGIC) {
        /* 首次使用，返回全零并标记无效 */
        memset(&state, 0, sizeof(state));
        return state;
    }

    memcpy(&state, (const void *)p, sizeof(state));

    /* 校验 CRC */
    uint32_t expected = calc_slot_state_crc(&state);
    if (state.crc != expected) {
        memset(&state, 0, sizeof(state));
    }

    return state;
}

/* ========== 写操作 ========== */
/*
 * Flash 写入流程: 解锁 → 擦页 → 半字编程 → 上锁
 * 注意: 擦除会暂停 CPU, DMA 环缓不受影响。
 */
void slot_set_state(const slot_state_t *state) {
    slot_state_t s;
    memcpy(&s, state, sizeof(s));
    s.magic = SLOT_STATE_MAGIC;
    s.crc   = calc_slot_state_crc(&s);

    FLASH_Unlock();
    FLASH_ErasePage(SLOT_STATE_ADDR);

    uint16_t *src = (uint16_t *)&s;
    for (uint32_t i = 0; i < sizeof(slot_state_t); i += 2) {
        FLASH_ProgramHalfWord(SLOT_STATE_ADDR + i, src[i / 2]);
    }

    FLASH_Lock();
}

/* ========== 槽位信息 ========== */

slot_t slot_get_active(void) {
    slot_state_t s = slot_get_state();
    if (s.magic == SLOT_STATE_MAGIC) {
        return (s.active_slot == SLOT_B) ? SLOT_B : SLOT_A;
    }
    return SLOT_A;   /* 默认 Slot A */
}

slot_t slot_get_inactive(void) {
    return (slot_get_active() == SLOT_A) ? SLOT_B : SLOT_A;
}

uint32_t slot_get_inactive_addr(void) {
    return SLOT_BASE(slot_get_inactive());
}

/* ========== 更新标志位 ========== */

void slot_set_enter_update(void) {
    slot_state_t s = slot_get_state();
    s.enter_update = 1;
    slot_set_state(&s);
}

void slot_clear_enter_update(void) {
    slot_state_t s = slot_get_state();
    s.enter_update = 0;
    slot_set_state(&s);
}

void slot_set_update_pending(void) {
    slot_state_t s = slot_get_state();
    slot_t inactive = slot_get_inactive();
    s.update_pending = 1;
    if (inactive == SLOT_A)
        s.slot_a_valid = 1;
    else
        s.slot_b_valid = 1;
    slot_set_state(&s);
}

void slot_switch_active(void) {
    slot_state_t s = slot_get_state();
    s.active_slot     = slot_get_inactive();
    s.update_pending  = 0;
    s.enter_update    = 0;
    slot_set_state(&s);
}

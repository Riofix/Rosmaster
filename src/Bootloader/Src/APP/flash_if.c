/**
 * @file flash_if.c
 * @brief Bootloader Flash 操作封装
 *
 *   STM32F103C8T6: 64KB Flash, 1KB/页, 半字(16-bit)编程
 *   Slot A: 页 8~35 (0x08002000 ~ 0x08008FFF, 28KB)
 *   Slot B: 页 36~63 (0x08009000 ~ 0x0800FFFF, 28KB)
 */

#include "flash_if.h"
#include "stm32f10x_flash.h"

/* ================================================================
 *  擦除 — 擦除整个槽位 (28 页 × 1KB)
 *  返回: 0=成功, -1=失败
 * ================================================================ */
int flash_erase_slot(slot_t slot) {
    uint32_t start = SLOT_BASE(slot);
    uint32_t end   = start + SLOT_A_SIZE;

    FLASH_Unlock();

    for (uint32_t addr = start; addr < end; addr += 0x400) {
        if (FLASH_ErasePage(addr) != FLASH_COMPLETE) {
            FLASH_Lock();
            return -1;
        }
    }

    FLASH_Lock();
    return 0;
}

/* ================================================================
 *  写入 — 按半字(16-bit)编程 Flash
 *  返回: 0=成功, -1=写入失败, -2=地址越界
 *
 *  注意: len 为奇数时最后 1 字节单独补 0xFF 写入
 * ================================================================ */
int flash_write_data(uint32_t addr, const uint8_t *data, uint32_t len) {
    if (len == 0) return 0;

    /* 地址范围检查 */
    if (addr < SLOT_A_BASE || addr + len > 0x08010000u) {
        return -2;
    }

    FLASH_Unlock();

    uint32_t i = 0;
    while (i + 1 < len) {
        uint16_t half = ((uint16_t)data[i + 1] << 8) | data[i];
        if (FLASH_ProgramHalfWord(addr + i, half) != FLASH_COMPLETE) {
            FLASH_Lock();
            return -1;
        }
        i += 2;
    }

    /* 处理奇数结尾: 最后一个字节 + 0xFF 补齐半字 */
    if (i < len) {
        uint16_t half = 0xFF00 | data[i];
        if (FLASH_ProgramHalfWord(addr + i, half) != FLASH_COMPLETE) {
            FLASH_Lock();
            return -1;
        }
    }

    FLASH_Lock();
    return 0;
}

/* ================================================================
 *  CRC32 — 对 Flash 指定范围计算 CRC32
 * ================================================================ */
uint32_t flash_crc32_range(uint32_t addr, uint32_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    const uint8_t *p = (const uint8_t *)addr;

    for (uint32_t i = 0; i < len; i++) {
        crc ^= p[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320u;
            else
                crc >>= 1;
        }
    }

    return ~crc;
}

/* ================================================================
 *  镜像头校验 — 检查 magic 和 image_size 是否合理
 * ================================================================ */
int flash_verify_image_header(slot_t slot) {
    uint32_t header_addr = IMAGE_HEADER_ADDR(slot);
    const image_header_t *h = (const image_header_t *)header_addr;

    if (h->magic != IMAGE_HEADER_MAGIC) {
        return -1;   /* 魔数不对 */
    }
    if (h->image_size == 0 || h->image_size > SLOT_A_SIZE) {
        return -2;   /* 大小不合理 */
    }
    return 0;
}

/* ================================================================
 *  整包 CRC 校验 — 与实际 Flash 内容比对 image_header.image_crc
 *  返回: 0=匹配, -1=CRC 不匹配, -2=镜像头无效
 * ================================================================ */
int flash_verify_image_crc(slot_t slot) {
    const image_header_t *h = (const image_header_t *)IMAGE_HEADER_ADDR(slot);

    if (h->magic != IMAGE_HEADER_MAGIC) {
        return -2;
    }

    uint32_t base   = SLOT_BASE(slot);
    uint32_t actual = flash_crc32_range(base, h->image_size);

    if (actual != h->image_crc) {
        return -1;
    }
    return 0;
}

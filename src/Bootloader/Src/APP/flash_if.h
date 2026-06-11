/**
 * @file flash_if.h
 * @brief Bootloader Flash 操作封装 — 擦除 / 写入 / 校验
 */

#ifndef __FLASH_IF_H
#define __FLASH_IF_H

#include "slot_manager.h"

/* ========== 擦除 ========== */
/* 擦除指定槽位全部页面 */
int flash_erase_slot(slot_t slot);

/* ========== 写入 ========== */
/* 向 Flash 写入一段数据 (自动对齐到半字) */
int flash_write_data(uint32_t addr, const uint8_t *data, uint32_t len);

/* ========== 校验 ========== */
/* 计算 Flash 某段范围的 CRC32 */
uint32_t flash_crc32_range(uint32_t addr, uint32_t len);

/* 校验槽位中镜像头是否有效 (magic + size) */
int flash_verify_image_header(slot_t slot);

/* 校验槽位中整包固件的 CRC (与 image_header.image_crc 比较) */
int flash_verify_image_crc(slot_t slot);

#endif /* __FLASH_IF_H */

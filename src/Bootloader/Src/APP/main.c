/**
 * @file main.c
 * @brief Bootloader 主流程
 *
 *   使用 bsp_systick 提供的 Delay_ms 轮询计时，不依赖 SysTick 中断。
 *
 *   流程:
 *   1. 硬件初始化
 *   2. 读 slot_state
 *   3. update_pending → 校验新固件 → 切换槽位
 *   4. enter_update → 擦除非活跃槽位 → 发 'R' → 接收固件 → 校验
 *   5. 正常启动 → 跳转 APP
 */

#include "stm32f10x.h"
#include "bsp_systick.h"
#include "bsp_usart.h"
#include "OLED.h"
#include "slot_manager.h"
#include "flash_if.h"

/* ================================================================
 *  jump_to_app — 跳转到 APP
 * ================================================================ */
static void jump_to_app(uint32_t app_addr) {
    __disable_irq();

    USART_Cmd(USART2, DISABLE);
    DMA_Cmd(DMA1_Channel6, DISABLE);

    __DSB();
    __ISB();

    uint32_t *vt = (uint32_t *)app_addr;
    __set_MSP(vt[0]);

    void (*reset)(void) = (void (*)(void))vt[1];
    reset();

    while (1);
}

/* ================================================================
 *  update_firmware — 接收固件 + 写入 Flash + 校验
 *  返回: 0=成功, -1=CRC 失败, -2=超时
 * ================================================================ */
static uint8_t num_digits(uint32_t n) {
    if (n == 0) return 1;
    uint8_t d = 0;
    while (n) { d++; n /= 10; }
    return d;
}

static int update_firmware(void) {
    slot_t   target = slot_get_inactive();
    uint32_t base   = SLOT_BASE(target);
    uint32_t addr   = base;
    uint32_t total  = 0;
    uint32_t imgsz  = 0;
    uint32_t silent = 0;
    uint8_t  buf[256];

    /* ---- 1. 擦除 ---- */
    OLED_Clear();
    OLED_ShowString(1, 1, "ERASING SLOT...");
    if (flash_erase_slot(target) != 0) {
        OLED_ShowString(2, 1, "ERASE FAIL!");
        return -2;
    }

    /* ---- 2. 通知 PC ---- */
    USART2_SendBuffer((uint8_t *)"ERASE OK\r\nREADY TO RECEIVE\r\n", 24);
    OLED_Clear();
    OLED_ShowString(1, 1, "RECEIVING...");

    /* ---- 3. 接收 ---- */
    while (1) {
        uint16_t avail = USART2_Available();
        if (avail > 0) {
            if (avail > sizeof(buf)) avail = sizeof(buf);
            USART2_Read(buf, avail);

            if (flash_write_data(addr, buf, avail) != 0) {
                OLED_ShowString(1, 1, "WRITE FAIL!");
                return -2;
            }

            addr   += avail;
            total  += avail;
            silent  = 0;

            /* 从 Flash 读取镜像头, 获取 image_size */
            if (imgsz == 0 && addr >= base + IMAGE_HEADER_OFFSET + sizeof(image_header_t)) {
                image_header_t *h = (image_header_t *)(base + IMAGE_HEADER_OFFSET);
                if (h->magic == IMAGE_HEADER_MAGIC)
                    imgsz = h->image_size;
            }

            /* L2: now / all */
            OLED_ShowNum(2, 1, total, num_digits(total));
            OLED_ShowChar(2, num_digits(total) + 1, '/');
            if (imgsz > 0) {
                OLED_ShowNum(2, num_digits(total) + 2, imgsz, num_digits(imgsz));
                OLED_ShowString(2, num_digits(total) + 2 + num_digits(imgsz), " B");

                /* L3: 百分比 */
                uint32_t pct = (total * 100) / imgsz;
                if (pct > 100) pct = 100;
                OLED_ShowNum(3, 1, pct, num_digits(pct));
                OLED_ShowChar(3, num_digits(pct) + 1, '%');
            } else {
                OLED_ShowString(2, num_digits(total) + 2, " ? B");
            }
        }

        Delay_ms(10);
        silent += 10;

        if (total > 0 && silent >= 3000) break;
        if (total == 0 && silent >= 30000) {
            USART2_SendBuffer((uint8_t *)"TIMEOUT NO DATA\r\n", 17);
            OLED_ShowString(1, 1, "TIMEOUT");
            return -2;
        }
    }

    /* ---- 4. 校验 ---- */
    OLED_ShowString(1, 1, "VERIFYING...");
    Delay_ms(100);

    if (flash_verify_image_header(target) != 0) {
        USART2_SendBuffer((uint8_t *)"BAD HEADER\r\n", 12);
        OLED_ShowString(1, 1, "BAD HEADER");
        return -1;
    }

    if (flash_verify_image_crc(target) != 0) {
        USART2_SendBuffer((uint8_t *)"CRC FAIL\r\n", 10);
        OLED_ShowString(1, 1, "CRC FAIL");
        return -1;
    }

    USART2_SendBuffer((uint8_t *)"CRC OK\r\n", 8);
    OLED_ShowString(1, 1, "CRC OK");
    return 0;
}

/* ================================================================
 *  main
 * ================================================================ */
int main(void) {
    /* ---- 硬件初始化 ---- */
    Systick_Init();
    USART2_Init(115200);
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 1, "BOOTLOADER V1.0");

    /* ---- 读取槽位状态 ---- */
    slot_state_t state = slot_get_state();

    /* ---- update_pending: 新固件待激活 ---- */
    if (state.magic == SLOT_STATE_MAGIC && state.update_pending) {
        slot_t new_slot = slot_get_inactive();
        OLED_ShowString(2, 1, "ACTIVATE NEW");

        if (flash_verify_image_header(new_slot) == 0 &&
            flash_verify_image_crc(new_slot) == 0) {
            slot_switch_active();
            OLED_ShowString(3, 1, "SWITCH OK");
        } else {
            OLED_ShowString(3, 1, "ACTIVATE FAIL");
            state.update_pending = 0;
            slot_set_state(&state);
        }
        Delay_ms(500);
    }

    /* ---- 判断是否进入更新模式 ---- */
    int do_update = 0;

    if (state.magic == SLOT_STATE_MAGIC && state.enter_update) {
        do_update = 1;
        OLED_ShowString(2, 1, "OTA MODE");
    }

    /* ---- 执行固件更新 ---- */
    if (do_update) {
        int retry = 0;
        while (retry < 3) {
            int result = update_firmware();

            if (result == 0) {
                USART2_SendBuffer((uint8_t *)"UPDATE SUCCESS\r\n", 16);
                OLED_ShowString(4, 1, "OK! RESET...");
                Delay_ms(500);
                slot_set_update_pending();
                NVIC_SystemReset();
            }

            retry++;
            USART2_SendBuffer((uint8_t *)"RETRY ", 6);
            USART2_SendByte('0' + retry);
            USART2_SendBuffer((uint8_t *)"/3\r\n", 4);

            OLED_ShowString(4, 1, "RETRY");
            OLED_ShowChar(4, 6, ' ');
            OLED_ShowNum(4, 7, retry, 1);
            OLED_ShowString(4, 8, "/3");
        }

        slot_clear_enter_update();
    }

    /* ---- 跳转 APP ---- */
    slot_t active = slot_get_active();
    uint32_t app_addr = SLOT_BASE(active);

    if (flash_verify_image_header(active) != 0) {
        OLED_Clear();
        OLED_ShowString(1, 1, "NO VALID APP!");
        OLED_ShowString(2, 1, "FLASH VIA SWD");
        while (1);
    }

    OLED_Clear();
    OLED_ShowString(1, 1, "JUMP TO APP");
    Delay_ms(200);
    jump_to_app(app_addr);

    return 0;
}

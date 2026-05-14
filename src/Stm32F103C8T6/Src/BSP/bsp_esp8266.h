#ifndef _BSP_ESP8266_H
#define _BSP_ESP8266_H

#include "stm32f10x.h"
#include "bsp_usart.h"
#include "bsp_systick.h"

/* ==================== 核心模式切换 ==================== */
#define ENABLE_AP                0   // 0: STA模式(连路由器), 1: AP模式

/* ==================== 底层硬件映射 ==================== */
#define ESP8266_SEND_DATA(buf, len)    USART2_SendBuffer((uint8_t*)buf, len)
#define ESP8266_RX_BUFFER_SIZE         USART2_RX_BUFFER_SIZE
#define ESP8266_DELAY_MS(ms)           Delay_ms(ms)

/* ==================== 网络配置参数 ==================== */
#if ENABLE_AP == 0
//    // STA 模式参数
//    #define WIFI_SSID       "Digua"
//    #define WIFI_PWD        "12345678"
//    #define SERVER_IP       "10.42.0.1" 
//    #define SERVER_PORT     "8080"
		// STA 模式参数
    // #define WIFI_SSID       "ll"
    // #define WIFI_PWD        "xy119125"
    // #define SERVER_IP       "192.168.43.167" 
    #define WIFI_SSID       "Redmi K70"
    #define WIFI_PWD        "44704470"
    #define SERVER_IP       "10.19.50.101" 
    #define SERVER_PORT     "3456"
#else
    // AP 模式参数
    #define AP_SSID         "STM32_Robot"
    #define AP_PWD          "12345678"
    #define AP_SERVER_IP    "192.168.4.2"
    #define AP_SERVER_PORT  "8080"
#endif

// 控制是否在每次通电时恢复 ESP8266 出厂设置
#define INIT_ON_BOOT 0

/* ==================== 接口声明 ==================== */
uint8_t ESP8266_Init(void);

#endif /* _BSP_ESP8266_H */

#ifndef _BSP_ESP8266_H
#define _BSP_ESP8266_H

#include "stm32f10x.h"
#include "bsp_usart.h" 

/* ==================== 核心模式切换 ==================== */
// 0: STA模式 (ESP连路由器)
// 1: AP模式  (ESP自己发热点，等待电脑连接)
#define ENABLE_AP                      0  

/* ==================== 移植映射区 ==================== */
#define ESP8266_SEND_DATA(buf, len)    USART2_SendBuffer(buf, len)
#define ESP8266_GET_RX_BUFFER()        USART2_GetRxBuffer()
#define ESP8266_RX_BUFFER_SIZE         USART2_RX_BUFFER_SIZE

extern void Delay_ms(uint16_t nms);
#define ESP8266_DELAY_MS(ms)           Delay_ms(ms)

#define ESP_TIMEOUT_GENERAL            1000   
#define ESP_TIMEOUT_WIFI               10000  
#define ESP_TIMEOUT_SERVER             5000   

/* ==================== 网络参数配置区 ==================== */

#if ENABLE_AP == 0
    // STA 模式参数 (连接现有路由器)
    #define WIFI_SSID       "Redmi K70"
    #define WIFI_PWD        "Color4470"
    #define SERVER_IP       "10.26.31.175" // 电脑在局域网的IP
    #define SERVER_PORT     "3456"
#else
    // AP 模式参数 (创建自己的热点)
    #define AP_SSID         "STM32_Robot"   // 发出的WIFI名字
    #define AP_PWD          "12345678"      // WIFI密码 (最少8位)
    #define AP_SERVER_IP    "192.168.4.2"   // 电脑连上ESP后默认被分配的IP
    #define AP_SERVER_PORT  "8080"
#endif

/* ==================================================== */

uint8_t ESP8266_Init_Transparent(void); // 参数已经全部通过宏定义搞定，不需要传参了

#endif

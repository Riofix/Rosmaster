#include "bsp_esp8266.h"
#include "OLED.h"
#include <stdio.h>
#include <string.h>

/* ==================== 内部私有函数声明 ==================== */
static uint8_t ESP8266_SendCmd(char *cmd, char *ack, uint32_t timeout);
static uint8_t ESP8266_Config(void);

/**
 * @brief 发送AT命令并等待应答
 * @param cmd AT命令字符串（包含\r\n），为NULL时仅检查缓冲区
 * @param ack 期望收到的应答字符串，为NULL时不等待应答
 * @param timeout 超时时间（毫秒）
 * @retval 1 成功收到期望应答，0 超时或未收到期望应答
 */
static uint8_t ESP8266_SendCmd(char *cmd, char *ack, uint32_t timeout) {
    char rx_buf[128];
    uint16_t rx_len = 0;
    memset(rx_buf, 0, sizeof(rx_buf));
    /* 清空接收缓冲区，避免旧数据干扰 */
    while (USART2_Available()) USART2_ReadByte();
    /* 发送命令（如果存在） */
    if (cmd != NULL) ESP8266_SEND_DATA((uint8_t *)cmd, strlen(cmd));
    /* 不需要等待应答时直接返回成功 */
    if (ack == NULL) return 1;
    /* 循环等待期望应答 */
    for (uint32_t i = 0; i < timeout; i++) {
        ESP8266_DELAY_MS(1);
        while (USART2_Available()) {
            char byte = (char)USART2_ReadByte();
            if (rx_len >= sizeof(rx_buf) - 1) {
                memmove(rx_buf, rx_buf + 32, 32);
                rx_len = 32;
            }
            rx_buf[rx_len++] = byte;
            rx_buf[rx_len] = '\0';
            if (strstr(rx_buf, ack) != NULL) return 1;
        }
    }
    return 0;
}

/**
 * @brief ESP8266模块配置（STA模式或AP模式）
 * @retval 1 配置成功，0 配置失败
 */
static uint8_t ESP8266_Config(void) {
    char cmd_buf[128];
    /* 步骤1：退出可能的透传模式 */
    ESP8266_DELAY_MS(200);
    ESP8266_SEND_DATA((uint8_t *)"+++", 3);
    ESP8266_DELAY_MS(1000);
    /* 步骤2：测试AT指令并关闭回显 */
    ESP8266_SendCmd("AT\r\n", "OK", 500);
    ESP8266_SendCmd("ATE0\r\n", "OK", 500);
#if INIT_ON_BOOT == 1
    /* 恢复出厂设置（可选） */
    if (!ESP8266_SendCmd("AT+RESTORE\r\n", "ready", 5000)) return 0;
    ESP8266_SendCmd("ATE0\r\n", "OK", 500);
#endif
#if ENABLE_AP == 0
    /* ========== STA模式配置 ========== */
    /* 步骤3：设置为STA模式 */
    if (!ESP8266_SendCmd("AT+CWMODE=1\r\n", "OK", 1000)) return 0;
    /* 步骤4：连接WiFi热点 */
    sprintf(cmd_buf, "AT+CWJAP=\"%s\",\"%s\"\r\n", WIFI_SSID, WIFI_PWD);
    if (!ESP8266_SendCmd(cmd_buf, "OK", 10000)) {
        /* 某些固件返回"GOT IP"表示连接成功 */
        if (!ESP8266_SendCmd(NULL, "GOT IP", 500)) return 0;
    }
    /* 步骤5：设置单连接模式 */
    ESP8266_SendCmd("AT+CIPMUX=0\r\n", "OK", 500);
    /* 步骤6：关闭已有连接 */
    ESP8266_SendCmd("AT+CIPCLOSE\r\n", NULL, 500);
    ESP8266_DELAY_MS(500);
    /* 步骤7：连接TCP服务器 */
    sprintf(cmd_buf, "AT+CIPSTART=\"TCP\",\"%s\",%s\r\n", SERVER_IP, SERVER_PORT);
    if (!ESP8266_SendCmd(cmd_buf, "OK", 5000)) {
        /* 可能已经连接成功 */
        if (!ESP8266_SendCmd(NULL, "ALREADY CONNECTED", 1000)) return 0;
    }
    /* 步骤8：开启透传模式 */
    if (!ESP8266_SendCmd("AT+CIPMODE=1\r\n", "OK", 1000)) return 0;
    /* 步骤9：开始透传发送 */
    if (!ESP8266_SendCmd("AT+CIPSEND\r\n", ">", 1000)) return 0;
#else
    /* ========== AP模式配置 ========== */
    /* 步骤3：设置为AP模式 */
    if (!ESP8266_SendCmd("AT+CWMODE=2\r\n", "OK", 1000)) return 0;
    /* 步骤4：配置AP参数 */
    sprintf(cmd_buf, "AT+CWSAP=\"%s\",\"%s\",1,3\r\n", AP_SSID, AP_PWD);
    if (!ESP8266_SendCmd(cmd_buf, "OK", 1000)) return 0;
    /* 步骤5：连接TCP服务器 */
    sprintf(cmd_buf, "AT+CIPSTART=\"TCP\",\"%s\",%s\r\n", AP_SERVER_IP, AP_SERVER_PORT);
    if (!ESP8266_SendCmd(cmd_buf, "OK", 5000)) return 0;
    /* 步骤6：开启透传模式 */
    if (!ESP8266_SendCmd("AT+CIPMODE=1\r\n", "OK", 1000)) return 0;
    /* 步骤7：开始透传发送 */
    if (!ESP8266_SendCmd("AT+CIPSEND\r\n", ">", 1000)) return 0;
#endif
    return 1;
}

/* ==================== 外部接口函数 ==================== */

/**
 * @brief ESP8266初始化（带重试机制）
 * @retval 1 初始化成功，0 初始化失败
 */
uint8_t ESP8266_Init(void) {
    uint8_t retry_count = 3;
    while (retry_count--) {
        if (ESP8266_Config() == 1) return 1;
        ESP8266_DELAY_MS(1000);
    }
    return 0;
}
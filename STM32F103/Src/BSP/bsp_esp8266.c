#include "bsp_esp8266.h"
#include <string.h>
#include <stdio.h>

static void ESP8266_ClearBuffer(void) {
    memset(ESP8266_GET_RX_BUFFER(), 0, ESP8266_RX_BUFFER_SIZE);
}

static uint8_t ESP8266_SendCmd(char* cmd, char* ack, uint32_t timeout) {
    ESP8266_ClearBuffer();
    ESP8266_SEND_DATA((uint8_t*)cmd, strlen(cmd));
    
    for (uint32_t i = 0; i < timeout; i++) {
        ESP8266_DELAY_MS(1); 
        if (strstr((char*)ESP8266_GET_RX_BUFFER(), ack) != NULL) {
            return 1; 
        }
    }
    return 0; 
}

/**
 * @brief  全自动配置ESP8266进入透传模式
 */
uint8_t ESP8266_Init_Transparent(void) {
    char cmd_buf[128];
    
    // 0. 退出可能存在的透传状态
    ESP8266_SEND_DATA((uint8_t*)"+++", 3);
    ESP8266_DELAY_MS(1000); 

    // 1. 握手测试
    if (!ESP8266_SendCmd("AT\r\n", "OK", ESP_TIMEOUT_GENERAL)) return 0;
    
#if ENABLE_AP == 0
    // ================= STA 模式配置 =================
    // 设置为 Station 模式
    if (!ESP8266_SendCmd("AT+CWMODE=1\r\n", "OK", ESP_TIMEOUT_GENERAL)) return 0;
    
    // 连接外部路由器
    sprintf(cmd_buf, "AT+CWJAP=\"%s\",\"%s\"\r\n", WIFI_SSID, WIFI_PWD);
    if (!ESP8266_SendCmd(cmd_buf, "OK", ESP_TIMEOUT_WIFI)) return 0; 
    
    // 连接 TCP 服务器
    sprintf(cmd_buf, "AT+CIPSTART=\"TCP\",\"%s\",%s\r\n", SERVER_IP, SERVER_PORT);
    if (!ESP8266_SendCmd(cmd_buf, "OK", ESP_TIMEOUT_SERVER)) return 0;

#else
    // ================= AP 模式配置 =================
    // 设置为 SoftAP 模式
    if (!ESP8266_SendCmd("AT+CWMODE=2\r\n", "OK", ESP_TIMEOUT_GENERAL)) return 0;
    
    // 配置热点参数 (通道1，WPA2_PSK加密方式)
    sprintf(cmd_buf, "AT+CWSAP=\"%s\",\"%s\",1,3\r\n", AP_SSID, AP_PWD);
    if (!ESP8266_SendCmd(cmd_buf, "OK", ESP_TIMEOUT_GENERAL)) return 0;
    
    // 等待电脑连接上这个热点并打开 TCP Server (此步骤可能会失败并重试)
    // 注意：电脑连上热点后，它的IP固定是 192.168.4.2
    sprintf(cmd_buf, "AT+CIPSTART=\"TCP\",\"%s\",%s\r\n", AP_SERVER_IP, AP_SERVER_PORT);
    if (!ESP8266_SendCmd(cmd_buf, "OK", ESP_TIMEOUT_SERVER)) return 0;
#endif

    // ================= 公共透传开启逻辑 =================
    // 开启透传模式标志位
    if (!ESP8266_SendCmd("AT+CIPMODE=1\r\n", "OK", ESP_TIMEOUT_GENERAL)) return 0;
    
    // 启动数据透传
    if (!ESP8266_SendCmd("AT+CIPSEND\r\n", ">", ESP_TIMEOUT_GENERAL)) return 0;
    
    ESP8266_ClearBuffer();
    
    return 1; // 成功！
}

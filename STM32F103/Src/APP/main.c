#include "OLED.h"
#include "bsp_esp8266.h"
#include "bsp_systick.h"
#include "bsp_usart.h"
#include "protocol.h"
#include "stm32f10x.h"
#include <stdio.h>

int main(void) {
  /* 基础硬件初始化 */
  Systick_Init();
  
  USART2_Init(115200);
  
  Delay_ms(1000);
  OLED_Init();

  OLED_Clear();
  OLED_ShowString(1, 1, "ESP8266 Boot...");
  OLED_ShowString(2, 1, "Connecting WiFi");
  OLED_ShowString(3, 1, "Please Wait... ");

  /* 3. 执行 ESP8266 智能配网与透传配置核心 */
  if (ESP8266_Init() == 1) {
    // ================= 配置成功 =================
    OLED_Clear();
    OLED_ShowString(1, 1, "WiFi Connected!");
    OLED_ShowString(2, 1, "TCP Server OK");
    OLED_ShowString(3, 1, "Pass-Thru Ready");
    OLED_ShowString(4, 1, "Waiting Data...");

    // 测试：透传通道建立后，主动向电脑的网络助手发一句话
    ESP8266_SEND_DATA((uint8_t *)"Hello Server! I am STM32.\r\n", 27);
  } else {
    // ================= 配置失败 =================
    OLED_Clear();
    OLED_ShowString(1, 1, "ESP Config FAIL");
    OLED_ShowString(2, 1, "1.Check AT+CWJAP");
    OLED_ShowString(3, 1, "2.Check TCP IP");
    OLED_ShowString(4, 1, "3.Reset Board");
  }

  /* 4. 主循环处理 */
  while (1) {
    // ==============================================================
    // 【测试案例】：网络透传连通性回环测试 (Echo Test)
    // 作用：从电脑端(网络助手)发字符到底板，底板立刻原样发回，并在OLED屏显示
    // ==============================================================
    while (USART2_Available() > 0) {
      // 1. 从新的环形缓冲区抽出一个字节数据
      uint8_t rx_byte = USART2_ReadByte();

      // 2. 立刻原路返回给 ESP8266（这就是通过WiFi跨空发给了您的电脑）
      USART2_SendByte(rx_byte);

      // 3. 将收到的字符实时刷新在 OLED 的第 4 行，直观确认收到数据
      char show_str[16];
      // 只显示可视字符以防乱码，非可视字符原样显示为十六进制
      if (rx_byte >= 32 && rx_byte <= 126) {
        sprintf(show_str, "Rx: '%c' (0x%02X)", rx_byte, rx_byte);
      } else {
        sprintf(show_str, "Rx: HEX (0x%02X)", rx_byte);
      }
      OLED_ShowString(4, 1, show_str);
    }

    // ==============================================================
    // 【正式业务逻辑】：当确认上面的 WiFi 测试通过后，
    // 您只需把上面的 回环测试 注释掉，然后把下面的 Protocol_Process(); 解开注释
    // 就可以走正式的电机控制协议了！
    // ==============================================================
    // Protocol_Process();
    // Protocol_Emm_Process();

    // 只要 10ms 的短延时，保证轮询的敏捷度，且不彻底锁死 CPU
    Delay_ms(10);
  }
}

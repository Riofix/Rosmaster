#include "OLED.h"
#include "app_protocol.h"
#include "bsp_esp8266.h"
#include "bsp_iic.h"
#include "bsp_pwm.h"
#include "bsp_systick.h"
#include "bsp_usart.h"
#include "protocol_emm.h"
#include "stm32f10x.h"
#include <stdio.h>

int main(void) {
  /* Hardware initialization */
  Systick_Init();

  USART_All_Init(115200, 115200);
	IIC_Init();
  MPU_Init();
  PWM_Init(); /* Initialize all 3 PWM channels (PA6/PB0/PB1) */
	
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

  /* 4. 底层协议与应用层初始化 */
  App_Protocol_Init();

  /* 5. 主循环处理 */
  while (1) {
    // ==============================================================
    // 【正式业务逻辑】：
    // 1. 底层解析数据封包进队列 (串口 -> 缓冲 -> 解析 -> 入队)
    // 2. 将下位机电机信息传递
    // 3. 应用层取出完整包执行建表分发
    // ==============================================================

    // 负责从 USART2 缓冲区抽取并解析，拼成包并 enqueue
    Protocol_Process();

    // 负责接收下位机反馈
    Protocol_Emm_Process();

    // 负责从队列中拿包并且映射到对应的处理函数中去执行
    App_Protocol_Tick();

    // 只要极短的延时，保证轮询的敏捷度，且不彻底锁死 CPU
    Delay_ms(2);
  }
}

#include "OLED.h"
#include "bsp_esp8266.h"
#include "bsp_systick.h"
#include "bsp_usart.h"
#include "stm32f10x.h"

int main(void) {
  // 1. 基础硬件初始化
  Systick_Init();
  USART2_Init(115200); // 初始化串口2接ESP8266
  OLED_Init();         // 初始化OLED

  // 2. 屏幕显示初始化中
  OLED_ShowString(1, 1, "Init...");

  // 3. 阻塞式连接网络
  OLED_ShowString(2, 1, "WiFi Connecting"); //
  while (1) {
    if (ESP8266_Init_Transparent() == 1) {
      OLED_Clear();                            // 连接成功，清屏
      OLED_ShowString(1, 1, "TCP Connected!"); //
      OLED_ShowString(2, 1, "Waiting Data:");  //
      break;
    } else {
      OLED_ShowString(3, 1, "Connect Failed "); //
      Delay_ms(2000);                           // 失败重试
    }
  }

  // 4. 主循环：读取透传数据并显示
  static uint16_t read_idx = 0;
  char display_buf[17]; // OLED一行最多显示16个字符，留一个给 '\0'
  uint8_t buf_len = 0;

  while (1) {
    uint8_t *rx_buf = USART2_GetRxBuffer();
    uint16_t write_idx = USART2_GetRxWriteIndex();

    // 检查串口 DMA 环形缓冲区是否有新数据
    while (read_idx != write_idx) {
      char c = rx_buf[read_idx];
      read_idx = (read_idx + 1) % USART2_RX_BUFFER_SIZE;

      // 将字符存入我们用来显示的数组中
      if (buf_len < 16) {
        display_buf[buf_len++] = c;
      }

      // 如果收到换行符('\n')，或者凑满了一行(16个字符)，就显示出来
      if (c == '\n' || buf_len == 16) {
        display_buf[buf_len] = '\0'; // 加上字符串结尾符

        // 清除第三行原来的内容（用空格覆盖）
        OLED_ShowString(3, 1, "                ");
        // 显示新收到的字符串
        OLED_ShowString(3, 1, display_buf);

        buf_len = 0; // 清空计数器，准备接收下一段
      }
    }
  }
}
// #include "stm32f10x.h"
// #include "bsp_systick.h"
// #include "OLED.h"
//// 请确保这里 include 了你的 IIC 头文件，比如 "bsp_iic.h"
// #include "bsp_iic.h"

// int main(void)
//{
//     // 1. 初始化最基础的时钟和引脚
//     Systick_Init();
//     IIC_Init();       // 如果你的名字叫 bsp_iic_init()，请换成对应的名字
//
//     // 2. 初始化 OLED
//     // 【关键点】：如果单片机死在这里，说明 100% 是 IIC
//     引脚配错或者线接反了！ OLED_Init();
//
//     // 3. 显示测试
//     OLED_Clear();
//     OLED_ShowString(1, 1, "OLED");
//     OLED_ShowString(2, 1, "SysTick");
//
//     // 4. 死循环
//     while(1)
//     {
//         // 如果能运行到这里，说明一点都没卡住
//     }
// }

/*
 * ==============================================================================
 * 【位置模式伺服/步进电机控制调用示例】
 *
 * 假设你想在上面的 WiFi 透传收到某条消息后，主动控制总线上地址为 0x01 的电机：
 *
 * #include "app_protocol.h"
 * #include "Emm_V5.h"
 *
 * void Control_Motor_Example(void) {
 *     // 1. (可选) 如果电机处于脱机释放状态，先开启锁轴使能
 *     Emm_V5_En_Control(0x01, true, false);
 *
 *     // 2. 发送全量位置模式控制指令:
 *     // - 0x01:   目标电机通信地址
 *     // - 0:      正转 (0:CW, 1:CCW)
 *     // - 500:    运行过程极速 500 RPM
 *     // - 0x80:   加速度 (0~255档位，0x80属于平滑适中)
 *     // - 10000:  移动 10000 个脉冲的距离
 *     // - false:  以当前位置计算相对运动(纯增量)；若传true则为绝对坐标模式
 *     // - false:  立刻单机执行 (传true则是放入缓存等待多机同步0x56指令触发)
 *     Emm_V5_Pos_Control(0x01, 0, 500, 0x80, 10000, false, false);
 *
 *     // 注意：实际项目中请不要在主轮询 while(1) 里不做条件判断死循环发送指令，
 *     //
 * 必须保证指令属于单步触发，否则电机会不停地被重新初始化位置而无法走出距离。
 * }
 *
 * ------------------------------------------------------------------------------
 * 【上位机 RDK X5 Python 下发数据包示例】
 *
 * 目标：STM32(0x0A)下的 1号电机(0x01)，正转(0x00)，500RPM(0x01F4)，
 *       加速度(0x80)，走 10000脉冲(0x00002710)。
 *
 * 组成按字节如下：
 *   FF 0A 52 0A 01 00 01 F4 80 00 00 27 10 00 12
 *
 * Python (pyserial) 写法：
 *
 * import serial
 * def send_pos_control(ser):
 *     # 构造 Data 域
 *     data = bytearray([
 *         0x01,                  # [0] Motor Addr
 *         0x00,                  # [1] Direction (0=CW)
 *         0x01, 0xF4,            # [2-3] Speed (16进制 0x01F4 = 500)
 *         0x80,                  # [4] Acceleration
 *         0x00, 0x00, 0x27, 0x10,# [5-8] Pulses (16进制 0x00002710 = 10000)
 *         0x00                   # [9] is_absolute (0=relative)
 *     ])
 *     # 组装 Header、ID、CMD、Len
 *     packet = bytearray([0xFF, 0x0A, 0x52, len(data)]) + data
 *     # 追加 1 Bytes 累加校验和
 *     packet.append(sum(packet) & 0xFF)
 *
 *     ser.write(packet)
 *
 * ==============================================================================
 */

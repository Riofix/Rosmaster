#include "stm32f10x.h"
#include "bsp_usart.h"
#include "bsp_esp8266.h"
#include "OLED.h"         
#include "bsp_systick.h"  

int main(void)
{
    // 1. 基础硬件初始化
    Systick_Init();
    USART2_Init(115200);  // 初始化串口2接ESP8266
    OLED_Init();          // 初始化OLED
    
    // 2. 屏幕显示初始化中
    OLED_ShowString(1, 1, "Init..."); 
    
    // 3. 阻塞式连接网络
    OLED_ShowString(2, 1, "WiFi Connecting"); //
    while (1) 
    {
        if (ESP8266_Init_Transparent() == 1) 
        {
            OLED_Clear();                         // 连接成功，清屏
            OLED_ShowString(1, 1, "TCP Connected!"); //
            OLED_ShowString(2, 1, "Waiting Data:");  //
            break; 
        } 
        else 
        {
            OLED_ShowString(3, 1, "Connect Failed "); //
            Delay_ms(2000); // 失败重试
        }
    }
    
    // 4. 主循环：读取透传数据并显示
    static uint16_t read_idx = 0;
    char display_buf[17]; // OLED一行最多显示16个字符，留一个给 '\0'
    uint8_t buf_len = 0;
    
    while(1)
    {
        uint8_t* rx_buf = USART2_GetRxBuffer();
        uint16_t write_idx = USART2_GetRxWriteIndex();
        
        // 检查串口 DMA 环形缓冲区是否有新数据
        while (read_idx != write_idx) 
        {
            char c = rx_buf[read_idx];
            read_idx = (read_idx + 1) % USART2_RX_BUFFER_SIZE;
            
            // 将字符存入我们用来显示的数组中
            if (buf_len < 16) {
                display_buf[buf_len++] = c;
            }
            
            // 如果收到换行符('\n')，或者凑满了一行(16个字符)，就显示出来
            if (c == '\n' || buf_len == 16) 
            {
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
//#include "stm32f10x.h"
//#include "bsp_systick.h"
//#include "OLED.h"
//// 请确保这里 include 了你的 IIC 头文件，比如 "bsp_iic.h"
//#include "bsp_iic.h" 

//int main(void)
//{
//    // 1. 初始化最基础的时钟和引脚
//    Systick_Init();
//    IIC_Init();       // 如果你的名字叫 bsp_iic_init()，请换成对应的名字
//    
//    // 2. 初始化 OLED
//    // 【关键点】：如果单片机死在这里，说明 100% 是 IIC 引脚配错或者线接反了！
//    OLED_Init();      
//    
//    // 3. 显示测试
//    OLED_Clear();
//    OLED_ShowString(1, 1, "OLED");
//    OLED_ShowString(2, 1, "SysTick");
//    
//    // 4. 死循环
//    while(1)
//    {
//        // 如果能运行到这里，说明一点都没卡住
//    }
//}

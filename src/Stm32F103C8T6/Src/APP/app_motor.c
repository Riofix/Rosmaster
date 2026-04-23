#include "app_motor.h"
#include "bsp_usart.h"
#include <string.h>

// 定义两个电机对象
MotorState_t g_motors[2];

// ======================= 解析函数定义 =======================

static void Parse_SystemState(uint8_t idx, uint8_t *data, uint8_t len)
{
    if (len < 28)
        return;

    // data[0]=1F (31 total bytes), data[1]=09 (param count)
    g_motors[idx].voltage_mv = (uint16_t)((data[2] << 8) | data[3]);
    g_motors[idx].phase_current = (int16_t)((data[4] << 8) | data[5]);
    g_motors[idx].encoder_val = (uint16_t)((data[6] << 8) | data[7]);

    // Target Pos (5 bytes: 1 sign + 4 val)
    int32_t tpos_val = (int32_t)((data[9] << 24) | (data[10] << 16) | (data[11] << 8) | data[12]);
    g_motors[idx].target_pos = (data[8] == 0x01) ? -tpos_val : tpos_val;

    // Velocity (3 bytes: 1 sign + 2 val)
    int16_t vel_val = (int16_t)((data[14] << 8) | data[15]);
    g_motors[idx].velocity = (data[13] == 0x01) ? -vel_val : vel_val;

    // Current Pos (5 bytes: 1 sign + 4 val)
    int32_t cpos_val = (int32_t)((data[17] << 24) | (data[18] << 16) | (data[19] << 8) | data[20]);
    g_motors[idx].current_pos = (data[16] == 0x01) ? -cpos_val : cpos_val;

    // Pos Error (5 bytes: 1 sign + 4 val)
    int32_t perr_val = (int32_t)((data[22] << 24) | (data[23] << 16) | (data[24] << 8) | data[25]);
    g_motors[idx].pos_error = (data[21] == 0x01) ? -perr_val : perr_val;

    // ORG flags (S_ORG) -> data[26]
    g_motors[idx].org = data[26];
    // uint8_t org = data[26];
    // g_motors[idx].enc_ready = (org & 0x01) != 0;
    // g_motors[idx].calib_ready = (org & 0x02) != 0;
    // g_motors[idx].is_homing = (org & 0x04) != 0;
    // g_motors[idx].home_failed = (org & 0x08) != 0;

    // FLAG flags (S_FLAG) -> data[27]
    g_motors[idx].flag = data[27];
    // uint8_t flag = data[27];
    // g_motors[idx].is_enabled = (flag & 0x01) != 0;
    // g_motors[idx].is_in_position = (flag & 0x02) != 0;
    // g_motors[idx].is_stalled = (flag & 0x04) != 0;
    // g_motors[idx].stall_protection = (flag & 0x08) != 0;
}

// ======================= 接口实现 =======================

void App_Motor_Init(void)
{
    memset(g_motors, 0, sizeof(g_motors));
    g_motors[0].addr = 1;
    g_motors[1].addr = 2;
    Protocol_Emm_Init(App_Motor_UpdateCallback);
}

void App_Motor_UpdateCallback(Emm_Feedback_t *msg)
{
    uint8_t idx = 0xFF;
    if (msg->addr == 1)
        idx = 0;
    else if (msg->addr == 2)
        idx = 1;

    if (idx == 0xFF)
        return;

    // 仅解析 0x43
    if (msg->func == 0x43)
    {
        Parse_SystemState(idx, msg->data, msg->len);
    }
}

// 拷贝结构体数据到其他的结构体中
void App_Motor_GetState(uint8_t idx, MotorState_t *state)
{
    if (idx < 2 && state != NULL)
    {
        *state = g_motors[idx];
    }
}

int32_t App_Motor_GetParam(uint8_t idx, uint8_t param)
{
    switch (param)
    {
    case 1:
        return g_motors[idx].voltage_mv;
    case 2:
        return (uint32_t)g_motors[idx].phase_current;
    case 3:
        return g_motors[idx].encoder_val;
    case 4:
        return (uint32_t)g_motors[idx].target_pos;
    case 5:
        return (uint32_t)g_motors[idx].velocity;
    case 6:
        return (uint32_t)g_motors[idx].current_pos;
    case 7:
        return (uint32_t)g_motors[idx].pos_error;
    case 8:
        return g_motors[idx].org;
    case 9:
        return g_motors[idx].flag;
    default:
        return 0;
    }
}

void App_Motor_RequestState(uint8_t addr)
{
    uint8_t cmd[4];
    cmd[0] = addr;
    cmd[1] = 0x43;
    cmd[2] = 0x7A;
    cmd[3] = 0x6B;
    // 使用 BSP 层串口驱动发送定长命令
    USART1_SendBuffer(cmd, 4);
}

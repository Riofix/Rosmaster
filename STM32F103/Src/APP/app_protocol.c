/**
 * @file app_protocol.c
 * @brief STM32 与 RDK X5 的核心业务交互逻辑层
 */

#include "app_protocol.h"
#include "stm32f10x_flash.h"
#include "Emm_V5.h"

uint8_t g_system_node_id = DEFAULT_SLAVE_ID;

/**
 * @brief 从Flash加载设备ID，若不存在则初始化
 */
static void Load_Device_ID(void) {
    uint32_t flash_val = *(__IO uint32_t*)FLASH_PARAM_ADDR;
    if (flash_val != 0xFFFFFFFF) {
        g_system_node_id = (uint8_t)(flash_val & 0xFF);
    } else {
        // 未初始化，擦除并写入默认值
        FLASH_Unlock();
        FLASH_ErasePage(FLASH_PARAM_ADDR);
        FLASH_ProgramWord(FLASH_PARAM_ADDR, DEFAULT_SLAVE_ID);
        FLASH_Lock();
        g_system_node_id = DEFAULT_SLAVE_ID;
    }
}

/**
 * @brief 更新设备ID到Flash
 */
static bool Save_Device_ID(uint8_t new_id) {
    FLASH_Unlock();
    FLASH_Status status = FLASH_ErasePage(FLASH_PARAM_ADDR);
    if (status == FLASH_COMPLETE) {
        status = FLASH_ProgramWord(FLASH_PARAM_ADDR, new_id);
    }
    FLASH_Lock();
    
    if (status == FLASH_COMPLETE) {
        g_system_node_id = new_id;
        return true;
    }
    return false;
}

/**
 * @brief 错误响应通用函数
 */
static void Send_Error_Ack(uint8_t motor_addr, uint8_t failed_cmd, uint8_t err_code) {
    uint8_t tx_data[3];
    tx_data[0] = motor_addr;
    tx_data[1] = failed_cmd;
    tx_data[2] = err_code;
    Protocol_PackAndSend(g_system_node_id, CMD_TX_ACK_ERR, tx_data, 3);
}

/**
 * @brief 成功响应通用函数
 */
static void Send_Ok_Ack(uint8_t motor_addr, uint8_t ok_cmd) {
    uint8_t tx_data[2];
    tx_data[0] = motor_addr;
    tx_data[1] = ok_cmd;
    Protocol_PackAndSend(g_system_node_id, CMD_TX_ACK_OK, tx_data, 2);
}

/**
 * @brief 协议初始化，读取Flash并注册回调
 */
void App_Protocol_Init(void) {
    Load_Device_ID();
    Protocol_Init(App_Protocol_Handler);
}

/**
 * @brief 收到完整收发包的处理回调映射中心
 */
void App_Protocol_Handler(Protocol_Packet_t* packet) {
    // 检查是否是发给本STM32节点的包
    // 0x00 保留为全局广播地址时也可以在此支持: packet->id != 0x00
    if (packet->id != g_system_node_id && packet->id != 0x00) {
        return; // 不是发给本机的指令，丢弃
    }

    uint8_t cmd = packet->cmd;
    
    // ============================ 板级专属控制 ============================
    // 如果是设置本板子ID的纯板级控制命令
    if (cmd == CMD_RX_SET_ID) {
        if (packet->len >= 1) {
            uint8_t new_id = packet->data[0];
            if (Save_Device_ID(new_id)) {
                Send_Ok_Ack(0, CMD_RX_SET_ID);
            } else {
                Send_Error_Ack(0, CMD_RX_SET_ID, ERR_FLASH_WRITE);
            }
        }
        return;
    }

    // ============================ 下方为具体电机透传 ============================
    // 对于电机相关命令，Data 第 0 字节必为电机 addr
    if (packet->len < 1) return; // 无电机地址，视为非法包抛弃
    uint8_t m_addr = packet->data[0];

    switch (cmd) {
        
        case CMD_RX_READ_PARAM: {
            if (packet->len >= 2) {
                uint8_t param_idx = packet->data[1];
                // 触发底层闭环状态轮询逻辑
                Emm_V5_Read_Sys_Params(m_addr, (SysParams_t)param_idx);
                // 注意: 此处仅代表解析下发成功反馈。真实底层电流、位置的读取返回值
                // 应当等从机(RS485/串口)返回后在另外的回调去触发 CMD_TX_ACK_PARAM。
                Send_Ok_Ack(m_addr, CMD_RX_READ_PARAM);
            } else {
                Send_Error_Ack(m_addr, CMD_RX_READ_PARAM, ERR_PARAM_INVALID);
            }
            break;
        }

        case CMD_RX_CTRL_SPEED: {
            // [0]addr, [1]dir, [2-3]vel, [4]acc
            if (packet->len >= 5) {
                uint8_t dir = packet->data[1];
                uint16_t vel = (packet->data[2] << 8) | packet->data[3];
                uint8_t acc = packet->data[4];
                // 第三参vel, 第四参acc, 末参表示是否启用多机同步snF
                Emm_V5_Vel_Control(m_addr, dir, vel, acc, false);
                Send_Ok_Ack(m_addr, CMD_RX_CTRL_SPEED);
            } else {
                Send_Error_Ack(m_addr, CMD_RX_CTRL_SPEED, ERR_PARAM_INVALID);
            }
            break;
        }

        case CMD_RX_CTRL_POS: {
            // [0]addr, [1]dir, [2-3]vel, [4]acc, [5-8]clk, [9]rel/abs
            if (packet->len >= 10) {
                uint8_t dir = packet->data[1];
                uint16_t vel = (packet->data[2] << 8) | packet->data[3];
                uint8_t acc = packet->data[4];
                uint32_t clk = (packet->data[5] << 24) | (packet->data[6] << 16) | (packet->data[7] << 8) | packet->data[8];
                bool is_abs = (packet->data[9] != 0);
                Emm_V5_Pos_Control(m_addr, dir, vel, acc, clk, is_abs, false);
                Send_Ok_Ack(m_addr, CMD_RX_CTRL_POS);
            } else {
                Send_Error_Ack(m_addr, CMD_RX_CTRL_POS, ERR_PARAM_INVALID);
            }
            break;
        }

        case CMD_RX_STOP: {
            // [0]addr, [1]释放动作(0急停, 1解堵转)
            if (packet->len >= 2) {
                if (packet->data[1] == 1) {
                    Emm_V5_Reset_Clog_Pro(m_addr);
                } else {
                    Emm_V5_Stop_Now(m_addr, false);
                }
                Send_Ok_Ack(m_addr, CMD_RX_STOP);
            } else {
                Send_Error_Ack(m_addr, CMD_RX_STOP, ERR_PARAM_INVALID);
            }
            break;
        }

        case CMD_RX_ENABLE: {
            if (packet->len >= 2) {
                bool state = (packet->data[1] != 0);
                Emm_V5_En_Control(m_addr, state, false);
                Send_Ok_Ack(m_addr, CMD_RX_ENABLE);
            } else {
                Send_Error_Ack(m_addr, CMD_RX_ENABLE, ERR_PARAM_INVALID);
            }
            break;
        }

        case CMD_RX_ORIGIN: {
            if (packet->len >= 2) {
                uint8_t o_mode = packet->data[1];
                Emm_V5_Origin_Trigger_Return(m_addr, o_mode, false);
                Send_Ok_Ack(m_addr, CMD_RX_ORIGIN);
            } else {
                Send_Error_Ack(m_addr, CMD_RX_ORIGIN, ERR_PARAM_INVALID);
            }
            break;
        }

        case CMD_RX_SYNC_RUN: {
            // [0]addr
            Emm_V5_Synchronous_motion(m_addr);
            Send_Ok_Ack(m_addr, CMD_RX_SYNC_RUN);
            break;
        }

        default:
            Send_Error_Ack(m_addr, cmd, ERR_UNKNOWN_CMD);
            break;
    }
}

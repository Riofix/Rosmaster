#ifndef __APP_RGB_H
#define __APP_RGB_H

#include <stdint.h>
#include "bsp_tcs3472.h"  // 引入底层驱动，复用原生的 TCS3472_RGBC_Data 结构体

/* ================== 算法配置参数 ================== */

// EMA 融合滤波权重配置 (建议 0-100 之间)
// FUSION_WEIGHT_NEW 越小，数据越平滑（抗突变能力越强），但响应速度会变慢
#define FUSION_WEIGHT_NEW  30  
#define FUSION_WEIGHT_OLD  (100 - FUSION_WEIGHT_NEW) 

// 豆子类型定义
typedef enum {
    BEAN_NONE   = 0,
    BEAN_WHITE  = 1,
    BEAN_YELLOW = 2,
    BEAN_GREEN  = 3,
    BEAN_STABLE_WAITING = 255 // 正在校准中
} Bean_Type_t;

/* ================== 业务数据结构体拆分 ================== */

/**
 * @brief 1. 基础净化层：经过滑动窗口去极值滤波后的纯净数据
 */
typedef struct {
    uint16_t red;
    uint16_t green;
    uint16_t blue;
    uint16_t clear;
} Rgbc_Clean_t;

/**
 * @brief 2. 瞬时特征层：归一化百分比与 HSV 解算数据 (反应灵敏，但可能有微小毛刺)
 */
typedef struct {
    uint8_t  red_percent;
    uint8_t  green_percent;
    uint8_t  blue_percent;
    
    uint16_t hue;
    uint8_t  saturation;
    uint8_t  value;
} Rgb_Feature_t;

/**
 * @brief 3. 终极平滑层：经过 EMA 融合滤波后的数据 (极其平滑，用于最终观察和阈值判定)
 */
typedef struct {
    uint8_t  red_percent;
    uint8_t  green_percent;
    uint8_t  blue_percent;
    
    uint16_t hue;
    uint8_t  saturation;
} Rgb_Fused_t;


/* ================== 应用层汇总结构体 ================== */

/**
 * @brief 应用层专属状态结构体：按生命周期组合，管理所有算法处理结果
 */
typedef struct {
    Rgbc_Clean_t  clean;   // 净化数据 (步骤 1)
    Rgb_Feature_t raw;     // 瞬时特征 (步骤 2)
    Rgb_Fused_t   fused;   // 融合特征 (步骤 3)
} App_Rgb_Data_t;



/* ================== 全局数据对象暴露 ================== */

// 1. 数据源对象：直接实例化底层 BSP 的结构体，用于接收最原始的传感数据
extern TCS3472_RGBC_Data g_raw_rgbc; 

// 2. 结果对象：应用层专属结构体，存放全链路算法处理后的清洗与特征数据
extern App_Rgb_Data_t g_app_rgb_data;


/* ================== 应用层 API ================== */

/**
 * @brief 初始化颜色传感器模块
 */
void App_Rgb_Init(void);

/**
 * @brief 核心数据处理流水线
 * @note  建议在 App_Tick 中定时调用 (例如 10ms 或 20ms 一次)。
 * @note  执行顺序：底层采集 -> 滑动滤波去极值 -> 归一化与HSV解算 -> EMA加权融合
 */
void App_Rgb_Update(void);

uint8_t App_Rgb_Get_Result(void);

uint8_t App_Rgb_Get_Validated_Result(void);

#endif /* __APP_RGB_H */

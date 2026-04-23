#ifndef __APP_RGB_H
#define __APP_RGB_H

#include <stdint.h>

/**
 * @brief 颜色传感器原始数据缓存
 */
typedef struct {
    uint16_t r;
    uint16_t g;
    uint16_t b;
    uint16_t c;           // 净光强度
} ColorRawData_t;

/**
 * @brief 颜色传感器识别结果缓存
 */
typedef struct {
    uint8_t color_id;     // 识别出的颜色编号
} ColorResult_t;

/**
 * @brief 初始化颜色传感器缓存 (预留)
 */
void App_Rgb_Init(void);

/**
 * @brief 获取颜色传感器原始数据 (预留)
 */
void App_Rgb_GetRaw(ColorRawData_t *raw);

/**
 * @brief 获取颜色传感器识别结果 (预留)
 */
void App_Rgb_GetResult(ColorResult_t *res);

#endif

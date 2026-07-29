#include "app_rgb.h"
#include <string.h>

/* ================================================================= */
/* 1. 全局变量实例化                                                 */
/* ================================================================= */

// 存放底层硬件原生数据的结构体 (Source)
TCS3472_RGBC_Data g_raw_rgbc = {0};

// 存放应用层全链路处理结果的结构体 (Context)
App_Rgb_Data_t g_app_rgb_data = {0};

/* ================================================================= */
/* 2. 私有函数前置声明 (Internal Pipeline)                           */
/* ================================================================= */

// 业务逻辑层函数
static void App_SlidingWindowFilter(void);
static void App_RGB_Ratio(void);
static void App_RGB2HSV(void);
static void App_EMA(void);

// 底层算法工具函数
static uint16_t Math_Median_Tool(uint16_t *buffer, uint8_t size);
static void Math_Convert_Tool(uint16_t r, uint16_t g, uint16_t b,
                              uint16_t *h, uint8_t *s, uint8_t *v);

/* ================================================================= */
/* 3. 公开 API 接口实现                                              */
/* ================================================================= */

/**
 * @brief 模块初始化
 */
void App_Rgb_Init(void)
{
    TCS3472_Init();
}

/**
 * @brief 颜色数据处理总调度更新
 * 顺序：采集 -> 滤波净化 -> 比例计算 -> 空间转换 -> 平滑融合
 */
void App_Rgb_Update(void)
{
    // 0. 读取原始 RGBC 寄存器数据
    TCS3472_ReadRGBC(&g_raw_rgbc);

    // 1. 第一步：滑动窗口滤波净化
    App_SlidingWindowFilter();

    // 2. 第二步：计算 RGB 归一化百分比
    App_RGB_Ratio();

    // 3. 第三步：计算 HSV 空间特征
    App_RGB2HSV();

    // 4. 第四步：执行 EMA 融合平滑处理
    App_EMA();
}

/* ================================================================= */
/* 4. 私有函数具体实现                                               */
/* ================================================================= */

/**
 * @brief 执行 7 样本滑动窗口中值滤波，将结果存入 clean 结构体
 */
static void App_SlidingWindowFilter(void)
{
    static uint16_t red_window[7], green_window[7], blue_window[7], clear_window[7];
    static uint8_t pointer = 0;

    // 数据入队
    red_window[pointer] = g_raw_rgbc.red;
    green_window[pointer] = g_raw_rgbc.green;
    blue_window[pointer] = g_raw_rgbc.blue;
    clear_window[pointer] = g_raw_rgbc.clear;

    if (++pointer >= 7)
        pointer = 0;

    // 取中值
    g_app_rgb_data.clean.red   = Math_Median_Tool(red_window, 7);
    g_app_rgb_data.clean.green = Math_Median_Tool(green_window, 7);
    g_app_rgb_data.clean.blue  = Math_Median_Tool(blue_window, 7);
    g_app_rgb_data.clean.clear = Math_Median_Tool(clear_window, 7);
}

/**
 * @brief 计算红、绿、蓝三色在当前环境下的占比 (百分比)
 */
static void App_RGB_Ratio(void)
{
    uint32_t total_intensity = g_app_rgb_data.clean.red +
                               g_app_rgb_data.clean.green +
                               g_app_rgb_data.clean.blue;

    if (total_intensity == 0)
        total_intensity = 1; // 避开除零错误

    g_app_rgb_data.raw.red_percent = (uint8_t)((g_app_rgb_data.clean.red * 100) / total_intensity);
    g_app_rgb_data.raw.green_percent = (uint8_t)((g_app_rgb_data.clean.green * 100) / total_intensity);
    g_app_rgb_data.raw.blue_percent = (uint8_t)((g_app_rgb_data.clean.blue * 100) / total_intensity);
}

/**
 * @brief 将净化后的 RGB 转换为 HSV 色彩空间特征
 */
static void App_RGB2HSV(void)
{
    Math_Convert_Tool(g_app_rgb_data.clean.red,
                      g_app_rgb_data.clean.green,
                      g_app_rgb_data.clean.blue,
                      &g_app_rgb_data.raw.hue,
                      &g_app_rgb_data.raw.saturation,
                      &g_app_rgb_data.raw.value);
}

/**
 * @brief 执行加权融合，使最终输出的数据趋势平滑
 */
static void App_EMA(void)
{
    static uint8_t is_initialized = 0;

    if (!is_initialized)
    {
        // 初次加载，直接同步瞬时值
        g_app_rgb_data.fused.red_percent = g_app_rgb_data.raw.red_percent;
        g_app_rgb_data.fused.green_percent = g_app_rgb_data.raw.green_percent;
        g_app_rgb_data.fused.blue_percent = g_app_rgb_data.raw.blue_percent;
        g_app_rgb_data.fused.hue = g_app_rgb_data.raw.hue;
        g_app_rgb_data.fused.saturation = g_app_rgb_data.raw.saturation;
        is_initialized = 1;
    }
    else
    {
        // 指数加权平均滤波
        g_app_rgb_data.fused.red_percent = (g_app_rgb_data.raw.red_percent * FUSION_WEIGHT_NEW + g_app_rgb_data.fused.red_percent * FUSION_WEIGHT_OLD) / 100;
        g_app_rgb_data.fused.green_percent = (g_app_rgb_data.raw.green_percent * FUSION_WEIGHT_NEW + g_app_rgb_data.fused.green_percent * FUSION_WEIGHT_OLD) / 100;
        g_app_rgb_data.fused.blue_percent = (g_app_rgb_data.raw.blue_percent * FUSION_WEIGHT_NEW + g_app_rgb_data.fused.blue_percent * FUSION_WEIGHT_OLD) / 100;

        g_app_rgb_data.fused.hue = (g_app_rgb_data.raw.hue * FUSION_WEIGHT_NEW + g_app_rgb_data.fused.hue * FUSION_WEIGHT_OLD) / 100;
        g_app_rgb_data.fused.saturation = (g_app_rgb_data.raw.saturation * FUSION_WEIGHT_NEW + g_app_rgb_data.fused.saturation * FUSION_WEIGHT_OLD) / 100;
    }
}

/**
 * @brief 数学工具：冒泡排序取中值
 * @param buffer 输入数组
 * @param size   数组大小 (奇数)
 * @return 中值
 */
static uint16_t Math_Median_Tool(uint16_t *buffer, uint8_t size)
{
    uint16_t sort_array[7];
    memcpy(sort_array, buffer, size * sizeof(uint16_t));

    // 冒泡排序
    for (uint8_t i = 0; i < size - 1; i++)
    {
        for (uint8_t j = 0; j < size - 1 - i; j++)
        {
            if (sort_array[j] > sort_array[j + 1])
            {
                uint16_t temp = sort_array[j];
                sort_array[j] = sort_array[j + 1];
                sort_array[j + 1] = temp;
            }
        }
    }
    // 返回中值 (size=7 -> index=3)
    return sort_array[size / 2];
}

/**
 * @brief 数学工具：RGB 到 HSV 定点数转换算法实现
 */
static void Math_Convert_Tool(uint16_t r, uint16_t g, uint16_t b,
                              uint16_t *h, uint8_t *s, uint8_t *v)
{
    uint32_t max_val = (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);
    uint32_t min_val = (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b);
    uint32_t difference = max_val - min_val;

    // 计算 Value (0-100)
    *v = (uint8_t)((max_val * 100) / 65535);

    if (max_val == 0)
    {
        *s = 0;
        *h = 0;
        return;
    }

    // 计算 Saturation (0-100)
    *s = (uint8_t)((difference * 100) / max_val);

    // 计算 Hue (0-360)
    if (difference == 0)
    {
        *h = 0;
    }
    else
    {
        int32_t h_temp = 0;
        if (max_val == r)
            h_temp = 60 * (g - b) / (int32_t)difference;
        else if (max_val == g)
            h_temp = 120 + 60 * (b - r) / (int32_t)difference;
        else
            h_temp = 240 + 60 * (r - g) / (int32_t)difference;

        if (h_temp < 0)
            h_temp += 360;
        *h = (uint16_t)h_temp;
    }
}

/**
 * @brief 根据实测 EMA 融合数据进行物体分类判定
 * @return uint8_t 识别到的豆子 ID (1:白, 2:黄, 3:绿, 0:未知)
 *
 * 判定策略 (以 Saturation 为核心分类器，Hue 为辅助确认):
 * ----------------------------------------------
 * |   类型   |    H      |    S      |  R%     |  B%     |
 * |----------|-----------|-----------|---------|---------|
 * |  白芸豆  |   >=90    |   22-30   |  25-35  |  22-30  |
 * |  黄豆    |   50-87   |   32-40   |  30-35  |  18-23  |
 * |  绿豆    |   45-90   |   40-50   |  32-42  |  14-22  |
 * ----------------------------------------------
 *   S 区间天然分隔: 白<=30 < 黄32-40 < 绿>=40, 间隙清晰可判
 */
//uint8_t App_Rgb_Get_Result(void)
//{
    //uint16_t h = g_app_rgb_data.fused.hue;
    //uint8_t s = g_app_rgb_data.fused.saturation;
    //uint16_t c = g_app_rgb_data.clean.clear;

    // 基础过滤：总亮度太低 -> 无豆
    //if (c < 300)
        //return BEAN_NONE;

    // 第1层：白芸豆 - H >= 90 + S <= 30（低饱和度，色相偏冷）
    //if (h >= 90 && s <= 30)
        //return BEAN_WHITE; // 3

    // 第2层：绿豆 - S >= 40（高饱和度，颜色最鲜艳）
    //if (s >= 40)
        //return BEAN_GREEN; // 2

    // 第3层：黄豆 - S 32-39（中等饱和度）+ H >= 50（暖色调确认）
    //if (s >= 32 && h >= 50)
        //return BEAN_YELLOW; // 1

    // 落入间隙区 -> 保守返回未知
    //return BEAN_NONE; // 0
//}

/**
 * @brief 根据实测 EMA 融合数据进行物体分类判定
 * @return uint8_t 识别到的豆子 ID (1:白, 2:黄, 3:绿, 0:未知)
 * * 判定依据 (基于用户实测数据):
 * - 白芸豆: Hue(120+), Saturation(10-25), BluePct(29)
 * - 黄豆:   Hue(60-75), Saturation(30-40), BluePct(25)
 * - 绿豆:   Hue(60-75), Saturation(40-50), BluePct(20)
 */

// uint8_t App_Rgb_Get_Result(void)
// {
//    uint16_t h = g_app_rgb_data.fused.hue;
//    uint8_t s = g_app_rgb_data.fused.saturation;
//    uint8_t b_pct = g_app_rgb_data.fused.blue_percent;
//    uint16_t c = g_app_rgb_data.clean.clear;

//    // 基础过滤：如果总亮度太低，认为没有放豆子
//    if (c < 300)
//        return BEAN_NONE;

//    // 逻辑 A: 判定白芸豆 (H特征明显)
//    if (h >= 100 && s <= 28)
//        return BEAN_WHITE; // 3

//    // 逻辑 B: 区分绿豆与黄豆 (利用 S 和 B% 的梯度差)
//    if (h >= 50 && h <= 85)
//    {
//        if (s > 40 && b_pct <= 22)
//            return BEAN_GREEN; // 2 绿豆
//        if (s <= 40 && b_pct > 22)
//            return BEAN_YELLOW; // 1 黄豆
//    }

//    // 逻辑 C: 冗余补偿 (根据 Clear 亮度做最后兜底)
//    if (c > 5000)
//        return BEAN_WHITE; // 3
//    if (c < 1800)
//        return BEAN_GREEN; // 2

//    return BEAN_NONE; // 0
// }

/**
 * @brief 根据实测 EMA 融合数据进行物体分类判定 (C-B 差值版)
 * @return uint8_t 识别到的豆子 ID (0:无, 1:黄豆, 2:绿豆, 3:白芸豆)
 *
 * 判定依据 (C-B 差值特征, 三区间天然不重叠):
 *   绿豆:   C-B 700-1300  + S 40-55  + H 55-82
 *   黄豆:   C-B 2000-2800 + S 30-35  + H 70-99
 *   白芸豆: C-B >3500     + S 20-35  + H 110-140
 *
 * 决策策略 (C-B 做主分类, H/S 交叉验证):
 *   C < 300                       -> NONE
 *   H >= 100  && C-B > 3000       -> WHITE
 *   C-B <= 1400 && S >= 36        -> GREEN
 *   C-B >= 1900 && S <= 37        -> YELLOW
 *   其他                          -> NONE
 */
//uint8_t App_Rgb_Get_Result(void)
//{
//    uint16_t h  = g_app_rgb_data.fused.hue;
//    uint8_t  s  = g_app_rgb_data.fused.saturation;
//    uint16_t c  = g_app_rgb_data.clean.clear;
//    uint16_t cb = c - g_app_rgb_data.clean.blue;
//
//    if (c < 300)
//        return BEAN_NONE;
//
//    if (h >= 100 && cb > 3000)
//        return BEAN_WHITE;
//
//    if (cb <= 1400 && s >= 36)
//        return BEAN_GREEN;
//
//    if (cb >= 1900 && s <= 37)
//        return BEAN_YELLOW;
//
//    return BEAN_NONE;
//}

/**
 * @brief 根据实测 RGBC 原始通道差值进行豆子分类判定
 * @return uint8_t 识别到的豆子 ID (0:无, 1:黄豆, 2:绿豆, 3:白芸豆)
 *
 * ── 实测数据范围 (2026-07-29 最新, 4组: 环境光/绿豆/黄豆/白豆) ──
 *   环境光:
 *     C: 468 (恒定, 极低)
 *   绿豆:
 *     C: 773-2952,  R-G: -510~-97,  G-B: 157-743,   R-B: -80~253
 *   黄豆:
 *     C: 682-8363,  R-G: -1462~-47, G-B: 105-1615,  R-B: -83~524
 *   白豆:
 *     C: 2943-10240,R-G: -3184~-424,G-B: 369-1845,  R-B: -1538~58
 *
 * ── 决策策略 (每种豆有独立判定条件) ──
 *   环境光: C < 600 (各豆下界 ≥ 682, 安全)
 *   白豆:   R-B ≤ -100  (白豆独有, 绿豆≥-80, 黄豆≥-83)
 *           R-G ≤ -1800 (白豆兜底, 黄豆≥-1462, 绿豆≥-510)
 *   黄豆:   R-G ≤ -600  (黄豆可到-1462, 绿豆最负-510)
 *           G-B ≥ 750   (黄豆可到1615, 绿豆最高743)
 *           R-B > 300   (黄豆可到524, 绿豆最高253)
 *   绿豆:   R-G ≥ -550  且  G-B ≤ 750  (绿豆专属区间)
 *   间隙:   其余 → NONE
 */
uint8_t App_Rgb_Get_Result(void)
{
    uint16_t c = g_app_rgb_data.clean.clear;
    uint16_t r = g_app_rgb_data.clean.red;
    uint16_t g = g_app_rgb_data.clean.green;
    uint16_t b = g_app_rgb_data.clean.blue;

    int16_t gb = (int16_t)(g - b);  /* G-B */
    int16_t rb = (int16_t)(r - b);  /* R-B */
    int16_t rg = (int16_t)(r - g);  /* R-G */

    /* ====== 环境光: C 极低 ====== */
    if (c < 600)
        return BEAN_NONE;        // 0

    /* ====== 白豆 (西配楼参数) ====== */
    if (rb <= -100)
       return BEAN_WHITE;       // 3
    /* 兜底: R-G 极负 (白豆可到-3184) */
    if (rg <= -1800)
       return BEAN_WHITE;       // 3

    /* ====== 白豆 (比赛参数) ======
     *   R-B ≤ -100  : 白豆独有负R-B区间 (新旧白豆均适用)
     *   R-G > 0     : 新白豆R>G独有 (绿豆≤-97, 黄豆≤-47)
     *   G-B < 0     : 新白豆B>G独有 (绿豆≥157, 黄豆≥105)
    //  *   兜底 R-G ≤ -1800 : 极负R-G (白豆可到-3184) */
    // if (rb <= -100 || rg > 0 || gb < 0)
    //     return BEAN_WHITE;       // 3
    // if (rg <= -1800)
    //     return BEAN_WHITE;       // 3

    /* ====== 黄豆: 三条独立特征 ====== */
    if (rg <= -600)              /* R-G 极度负偏, 绿豆最负-510 */
        return BEAN_YELLOW;      // 1
    if (gb >= 750)               /* G-B 高值, 绿豆最高743 */
        return BEAN_YELLOW;      // 1
    if (rb > 300)                /* R-B 高值, 绿豆最高253 */
        return BEAN_YELLOW;      // 1

    /* ====== 绿豆: R-G不极负 + G-B不过高 ====== */
    if (rg >= -550 && gb <= 750)
        return BEAN_GREEN;       // 2

    /* ====== 间隙区 ====== */
    return BEAN_NONE;            // 0
}


/**
 * @brief 带置信度校验的结果输出函数
 * @return 0:无效/等待, 1-3:确定的豆子ID
 */
uint8_t App_Rgb_Get_Validated_Result(void)
{
    static uint8_t last_raw_id = 0;     // 上一次的瞬时识别结果
    static uint16_t confidence_cnt = 0; // 置信度计数器
    static uint8_t final_stable_id = 0; // 最终锁定的ID

    // 1. 获取当前瞬时的识别结果 (逻辑 C)
    uint8_t current_raw_id = App_Rgb_Get_Result();

    // 2. 如果当前识别为空 (没放豆子)，则重置一切状态
    if (current_raw_id == BEAN_NONE)
    {
        last_raw_id = BEAN_NONE;
        confidence_cnt = 0;
        final_stable_id = BEAN_NONE;
        return BEAN_NONE;
    }

    // 3. 如果当前识别与上一次一致
    if (current_raw_id == last_raw_id)
    {
        if (confidence_cnt < 20)
        { // 阈值设定：连续 20 次一致 (约 200ms)
            confidence_cnt++;
        }
        else
        {
            // 达到置信度要求，锁定最终输出
            final_stable_id = current_raw_id;
        }
    }
    // 4. 如果当前识别与上一次不一致 (说明数据还在跳动或换了豆子)
    else
    {
        confidence_cnt = 0; // 计数器清零，重新开始计时
        last_raw_id = current_raw_id;
        // 注意：这里不立刻清空 final_stable_id，防止短暂毛刺导致输出中断
    }

    // 5. 返回结果：只有达到置信度才返回真实 ID，否则返回等待状态
    return (confidence_cnt >= 20) ? final_stable_id : BEAN_STABLE_WAITING;
}

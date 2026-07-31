#include "app_rgb.h"
#include <string.h>

/* ================================================================= */
/* 1. 鍏ㄥ眬鍙橀噺瀹炰緥锟?                                                */
/* ================================================================= */

// 瀛樻斁搴曞眰纭欢鍘熺敓鏁版嵁鐨勭粨鏋勪綋 (Source)
TCS3472_RGBC_Data g_raw_rgbc = {0};

// 瀛樻斁搴旂敤灞傚叏閾捐矾澶勭悊缁撴灉鐨勭粨鏋勪綋 (Context)
App_Rgb_Data_t g_app_rgb_data = {0};

/* ================================================================= */
/* 2. 绉佹湁鍑芥暟鍓嶇疆澹版槑 (Internal Pipeline)                           */
/* ================================================================= */

// 涓氬姟閫昏緫灞傚嚱锟?
static void App_SlidingWindowFilter(void);
static void App_RGB_Ratio(void);
static void App_RGB2HSV(void);
static void App_EMA(void);
static void App_SlidingAlphaBeta(void);

// 搴曞眰绠楁硶宸ュ叿鍑芥暟
static uint16_t Math_Median_Tool(uint16_t *buffer, uint8_t size);
static void Math_Convert_Tool(uint16_t r, uint16_t g, uint16_t b,
                              uint16_t *h, uint8_t *s, uint8_t *v);

/* ================================================================= */
/* 3. 鍏紑 API 鎺ュ彛瀹炵幇                                              */
/* ================================================================= */

/**
 * @brief 妯″潡鍒濆锟?
 */
void App_Rgb_Init(void)
{
    TCS3472_Init();
}

/**
 * @brief 棰滆壊鏁版嵁澶勭悊鎬昏皟搴︽洿锟?
 * 椤哄簭锛氶噰锟?-> 婊ゆ尝鍑€锟?-> 姣斾緥璁＄畻 -> 绌洪棿杞崲 -> 骞虫粦铻嶅悎
 */
void App_Rgb_Update(void)
{
    // 0. 璇诲彇鍘熷 RGBC 瀵勫瓨鍣ㄦ暟锟?
    TCS3472_ReadRGBC(&g_raw_rgbc);

    // 1. 绗竴姝ワ細婊戝姩绐楀彛婊ゆ尝鍑€锟?
    App_SlidingWindowFilter();

    // 2. 绗簩姝ワ細璁＄畻 RGB 褰掍竴鍖栫櫨鍒嗘瘮
    App_RGB_Ratio();

    // 3. 绗笁姝ワ細璁＄畻 HSV 绌洪棿鐗瑰緛
    App_RGB2HSV();

    // 4. 绗洓姝ワ細鎵ц EMA 铻嶅悎骞虫粦澶勭悊
    App_EMA();

    // 5. 绗簲姝ワ細伪/尾 婊戝姩绐楀彛鏇存柊
    App_SlidingAlphaBeta();
}

/* ================================================================= */
/* 4. 绉佹湁鍑芥暟鍏蜂綋瀹炵幇                                               */
/* ================================================================= */

/**
 * @brief 鎵ц 7 鏍锋湰婊戝姩绐楀彛涓€兼护娉紝灏嗙粨鏋滃瓨锟?clean 缁撴瀯锟?
 */
static void App_SlidingWindowFilter(void)
{
    static uint16_t red_window[7], green_window[7], blue_window[7], clear_window[7];
    static uint8_t pointer = 0;

    // 鏁版嵁鍏ラ槦
    red_window[pointer] = g_raw_rgbc.red;
    green_window[pointer] = g_raw_rgbc.green;
    blue_window[pointer] = g_raw_rgbc.blue;
    clear_window[pointer] = g_raw_rgbc.clear;

    if (++pointer >= 7)
        pointer = 0;

    // 鍙栦腑锟?
    g_app_rgb_data.clean.red = Math_Median_Tool(red_window, 7);
    g_app_rgb_data.clean.green = Math_Median_Tool(green_window, 7);
    g_app_rgb_data.clean.blue = Math_Median_Tool(blue_window, 7);
    g_app_rgb_data.clean.clear = Math_Median_Tool(clear_window, 7);
}

/**
 * @brief 璁＄畻绾€佺豢銆佽摑涓夎壊鍦ㄥ綋鍓嶇幆澧冧笅鐨勫崰锟?(鐧惧垎锟?
 */
static void App_RGB_Ratio(void)
{
    uint32_t total_intensity = g_app_rgb_data.clean.red +
                               g_app_rgb_data.clean.green +
                               g_app_rgb_data.clean.blue;

    if (total_intensity == 0)
        total_intensity = 1; // 閬垮紑闄ら浂閿欒

    g_app_rgb_data.raw.red_percent = (uint8_t)((g_app_rgb_data.clean.red * 100) / total_intensity);
    g_app_rgb_data.raw.green_percent = (uint8_t)((g_app_rgb_data.clean.green * 100) / total_intensity);
    g_app_rgb_data.raw.blue_percent = (uint8_t)((g_app_rgb_data.clean.blue * 100) / total_intensity);
}

/**
 * @brief 灏嗗噣鍖栧悗锟?RGB 杞崲锟?HSV 鑹插僵绌洪棿鐗瑰緛
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
 * @brief 鎵ц鍔犳潈铻嶅悎锛屼娇鏈€缁堣緭鍑虹殑鏁版嵁瓒嬪娍骞虫粦
 */
static void App_EMA(void)
{
    static uint8_t is_initialized = 0;

    if (!is_initialized)
    {
        // 鍒濇鍔犺浇锛岀洿鎺ュ悓姝ョ灛鏃讹拷?
        g_app_rgb_data.fused.red_percent = g_app_rgb_data.raw.red_percent;
        g_app_rgb_data.fused.green_percent = g_app_rgb_data.raw.green_percent;
        g_app_rgb_data.fused.blue_percent = g_app_rgb_data.raw.blue_percent;
        g_app_rgb_data.fused.hue = g_app_rgb_data.raw.hue;
        g_app_rgb_data.fused.saturation = g_app_rgb_data.raw.saturation;
        is_initialized = 1;
    }
    else
    {
        // 鎸囨暟鍔犳潈骞冲潎婊ゆ尝
        g_app_rgb_data.fused.red_percent = (g_app_rgb_data.raw.red_percent * FUSION_WEIGHT_NEW + g_app_rgb_data.fused.red_percent * FUSION_WEIGHT_OLD) / 100;
        g_app_rgb_data.fused.green_percent = (g_app_rgb_data.raw.green_percent * FUSION_WEIGHT_NEW + g_app_rgb_data.fused.green_percent * FUSION_WEIGHT_OLD) / 100;
        g_app_rgb_data.fused.blue_percent = (g_app_rgb_data.raw.blue_percent * FUSION_WEIGHT_NEW + g_app_rgb_data.fused.blue_percent * FUSION_WEIGHT_OLD) / 100;

        g_app_rgb_data.fused.hue = (g_app_rgb_data.raw.hue * FUSION_WEIGHT_NEW + g_app_rgb_data.fused.hue * FUSION_WEIGHT_OLD) / 100;
        g_app_rgb_data.fused.saturation = (g_app_rgb_data.raw.saturation * FUSION_WEIGHT_NEW + g_app_rgb_data.fused.saturation * FUSION_WEIGHT_OLD) / 100;
    }
}

/**
 * @brief 鏁板宸ュ叿锛氬啋娉℃帓搴忓彇涓拷?
 * @param buffer 杈撳叆鏁扮粍
 * @param size   鏁扮粍澶у皬 (濂囨暟)
 * @return 涓拷?
 */
static uint16_t Math_Median_Tool(uint16_t *buffer, uint8_t size)
{
    uint16_t sort_array[7];
    memcpy(sort_array, buffer, size * sizeof(uint16_t));

    // 鍐掓场鎺掑簭
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
    // 杩斿洖涓拷?(size=7 -> index=3)
    return sort_array[size / 2];
}

/**
 * @brief 鏁板宸ュ叿锛歊GB 锟?HSV 瀹氱偣鏁拌浆鎹㈢畻娉曞疄锟?
 */
static void Math_Convert_Tool(uint16_t r, uint16_t g, uint16_t b,
                              uint16_t *h, uint8_t *s, uint8_t *v)
{
    uint32_t max_val = (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);
    uint32_t min_val = (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b);
    uint32_t difference = max_val - min_val;

    // 璁＄畻 Value (0-100)
    *v = (uint8_t)((max_val * 100) / 65535);

    if (max_val == 0)
    {
        *s = 0;
        *h = 0;
        return;
    }

    // 璁＄畻 Saturation (0-100)
    *s = (uint8_t)((difference * 100) / max_val);

    // 璁＄畻 Hue (0-360)
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
 * @brief 鏍规嵁瀹炴祴 EMA 铻嶅悎鏁版嵁杩涜鐗╀綋鍒嗙被鍒ゅ畾
 * @return uint8_t 璇嗗埆鍒扮殑璞嗗瓙 ID (1:锟? 2:锟? 3:锟? 0:鏈煡)
 *
 * 鍒ゅ畾绛栫暐 (锟?Saturation 涓烘牳蹇冨垎绫诲櫒锛孒ue 涓鸿緟鍔╃‘锟?:
 * ----------------------------------------------
 * |   绫诲瀷   |    H      |    S      |  R%     |  B%     |
 * |----------|-----------|-----------|---------|---------|
 * |  鐧借姼锟? |   >=90    |   22-30   |  25-35  |  22-30  |
 * |  榛勮眴    |   50-87   |   32-40   |  30-35  |  18-23  |
 * |  缁胯眴    |   45-90   |   40-50   |  32-42  |  14-22  |
 * ----------------------------------------------
 *   S 鍖洪棿澶╃劧鍒嗛殧: 锟?=30 < 锟?2-40 < 锟?=40, 闂撮殭娓呮櫚鍙垽
 */
// uint8_t App_Rgb_Get_Result(void)
//{
// uint16_t h = g_app_rgb_data.fused.hue;
// uint8_t s = g_app_rgb_data.fused.saturation;
// uint16_t c = g_app_rgb_data.clean.clear;

// 鍩虹杩囨护锛氭€讳寒搴﹀お锟?-> 鏃犺眴
// if (c < 300)
// return BEAN_NONE;

// 锟?灞傦細鐧借姼锟?- H >= 90 + S <= 30锛堜綆楗卞拰搴︼紝鑹茬浉鍋忓喎锟?
// if (h >= 90 && s <= 30)
// return BEAN_WHITE; // 3

// 锟?灞傦細缁胯眴 - S >= 40锛堥珮楗卞拰搴︼紝棰滆壊鏈€椴滆壋锟?
// if (s >= 40)
// return BEAN_GREEN; // 2

// 锟?灞傦細榛勮眴 - S 32-39锛堜腑绛夐ケ鍜屽害锟? H >= 50锛堟殩鑹茶皟纭锟?
// if (s >= 32 && h >= 50)
// return BEAN_YELLOW; // 1

// 钀藉叆闂撮殭锟?-> 淇濆畧杩斿洖鏈煡
// return BEAN_NONE; // 0
//}

/**
 * @brief 鏍规嵁瀹炴祴 EMA 铻嶅悎鏁版嵁杩涜鐗╀綋鍒嗙被鍒ゅ畾
 * @return uint8_t 璇嗗埆鍒扮殑璞嗗瓙 ID (1:锟? 2:锟? 3:锟? 0:鏈煡)
 * * 鍒ゅ畾渚濇嵁 (鍩轰簬鐢ㄦ埛瀹炴祴鏁版嵁):
 * - 鐧借姼锟? Hue(120+), Saturation(10-25), BluePct(29)
 * - 榛勮眴:   Hue(60-75), Saturation(30-40), BluePct(25)
 * - 缁胯眴:   Hue(60-75), Saturation(40-50), BluePct(20)
 */

// uint8_t App_Rgb_Get_Result(void)
//{
//     uint16_t h = g_app_rgb_data.fused.hue;
//     uint8_t s = g_app_rgb_data.fused.saturation;
//     uint8_t b_pct = g_app_rgb_data.fused.blue_percent;
//     uint16_t c = g_app_rgb_data.clean.clear;
//
//     // 鍩虹杩囨护锛氬鏋滄€讳寒搴﹀お浣庯紝璁や负娌℃湁鏀捐眴锟?
//     if (c < 300)
//         return BEAN_NONE;
//
//     // 閫昏緫 A: 鍒ゅ畾鐧借姼锟?(H鐗瑰緛鏄庢樉)
//     if (h >= 100 && s <= 28)
//         return BEAN_WHITE; // 3
//
//     // 閫昏緫 B: 鍖哄垎缁胯眴涓庨粍锟?(鍒╃敤 S 锟?B% 鐨勬搴﹀樊)
//     if (h >= 50 && h <= 85)
//     {
//         if (s > 40 && b_pct <= 22)
//             return BEAN_GREEN; // 2 缁胯眴
//         if (s <= 40 && b_pct > 22)
//             return BEAN_YELLOW; // 1 榛勮眴
//     }
//
//     // 閫昏緫 C: 鍐椾綑琛ュ伩 (鏍规嵁 Clear 浜害鍋氭渶鍚庡厹锟?
//     if (c > 5000)
//         return BEAN_WHITE; // 3
//     if (c < 1800)
//         return BEAN_GREEN; // 2
//
//     return BEAN_NONE; // 0
// {
//    uint16_t h = g_app_rgb_data.fused.hue;
//    uint8_t s = g_app_rgb_data.fused.saturation;
//    uint8_t b_pct = g_app_rgb_data.fused.blue_percent;
//    uint16_t c = g_app_rgb_data.clean.clear;

//    // 鍩虹杩囨护锛氬鏋滄€讳寒搴﹀お浣庯紝璁や负娌℃湁鏀捐眴锟?
//    if (c < 300)
//        return BEAN_NONE;

//    // 閫昏緫 A: 鍒ゅ畾鐧借姼锟?(H鐗瑰緛鏄庢樉)
//    if (h >= 100 && s <= 28)
//        return BEAN_WHITE; // 3

//    // 閫昏緫 B: 鍖哄垎缁胯眴涓庨粍锟?(鍒╃敤 S 锟?B% 鐨勬搴﹀樊)
//    if (h >= 50 && h <= 85)
//    {
//        if (s > 40 && b_pct <= 22)
//            return BEAN_GREEN; // 2 缁胯眴
//        if (s <= 40 && b_pct > 22)
//            return BEAN_YELLOW; // 1 榛勮眴
//    }

//    // 閫昏緫 C: 鍐椾綑琛ュ伩 (鏍规嵁 Clear 浜害鍋氭渶鍚庡厹锟?
//    if (c > 5000)
//        return BEAN_WHITE; // 3
//    if (c < 1800)
//        return BEAN_GREEN; // 2

//    return BEAN_NONE; // 0
// }

/**
 * @brief 鏍规嵁瀹炴祴 EMA 铻嶅悎鏁版嵁杩涜鐗╀綋鍒嗙被鍒ゅ畾 (C-B 宸€肩増)
 * @return uint8_t 璇嗗埆鍒扮殑璞嗗瓙 ID (0:锟? 1:榛勮眴, 2:缁胯眴, 3:鐧借姼锟?
 *
 * 鍒ゅ畾渚濇嵁 (C-B 宸€肩壒锟? 涓夊尯闂村ぉ鐒朵笉閲嶅彔):
 *   缁胯眴:   C-B 700-1300  + S 40-55  + H 55-82
 *   榛勮眴:   C-B 2000-2800 + S 30-35  + H 70-99
 *   鐧借姼锟? C-B >3500     + S 20-35  + H 110-140
 *
 * 鍐崇瓥绛栫暐 (C-B 鍋氫富鍒嗙被, H/S 浜ゅ弶楠岃瘉):
 *   C < 300                       -> NONE
 *   H >= 100  && C-B > 3000       -> WHITE
 *   C-B <= 1400 && S >= 36        -> GREEN
 *   C-B >= 1900 && S <= 37        -> YELLOW
 *   鍏朵粬                          -> NONE
 */
// uint8_t App_Rgb_Get_Result(void)
//{
//     uint16_t h  = g_app_rgb_data.fused.hue;
//     uint8_t  s  = g_app_rgb_data.fused.saturation;
//     uint16_t c  = g_app_rgb_data.clean.clear;
//     uint16_t cb = c - g_app_rgb_data.clean.blue;
//
//     if (c < 300)
//         return BEAN_NONE;
//
//     if (h >= 100 && cb > 3000)
//         return BEAN_WHITE;
//
//     if (cb <= 1400 && s >= 36)
//         return BEAN_GREEN;
//
//     if (cb >= 1900 && s <= 37)
//         return BEAN_YELLOW;
//
//     return BEAN_NONE;
// }

/**
 * @brief 鏍规嵁瀹炴祴 RGBC 鍘熷閫氶亾宸€艰繘琛岃眴瀛愬垎绫诲垽锟?
 * @return uint8_t 璇嗗埆鍒扮殑璞嗗瓙 ID (0:锟? 1:榛勮眴, 2:缁胯眴, 3:鐧借姼锟?
 *
 * 鈹€鈹€ 瀹炴祴鏁版嵁鑼冨洿 (2026-07-29 鏈€锟? 4锟? 鐜锟?缁胯眴/榛勮眴/鐧借眴) 鈹€鈹€
 *   鐜锟?
 *     C: 468 (鎭掑畾, 鏋佷綆)
 *   缁胯眴:
 *     C: 773-2952,  R-G: -510~-97,  G-B: 157-743,   R-B: -80~253
 *   榛勮眴:
 *     C: 682-8363,  R-G: -1462~-47, G-B: 105-1615,  R-B: -83~524
 *   鐧借眴:
 *     C: 2943-10240,R-G: -3184~-424,G-B: 369-1845,  R-B: -1538~58
 *
 * 鈹€鈹€ 鍐崇瓥绛栫暐 (姣忕璞嗘湁鐙珛鍒ゅ畾鏉′欢) 鈹€鈹€
 *   鐜锟? C < 600 (鍚勮眴涓嬬晫 锟?682, 瀹夊叏)
 *   鐧借眴:   R-B 锟?-100  (鐧借眴鐙湁, 缁胯眴锟?80, 榛勮眴锟?83)
 *           R-G 锟?-1800 (鐧借眴鍏滃簳, 榛勮眴锟?1462, 缁胯眴锟?510)
 *   榛勮眴:   R-G 锟?-600  (榛勮眴鍙埌-1462, 缁胯眴鏈€锟?510)
 *           G-B 锟?750   (榛勮眴鍙埌1615, 缁胯眴鏈€锟?43)
 *           R-B > 300   (榛勮眴鍙埌524, 缁胯眴鏈€锟?53)
 *   缁胯眴:   R-G 锟?-550  锟? G-B 锟?750  (缁胯眴涓撳睘鍖洪棿)
 *   闂撮殭:   鍏朵綑 锟?NONE
 */
// /**
//  * @brief 鏍规嵁瀹炴祴 RGBC 鍘熷閫氶亾宸€艰繘琛岃眴瀛愬垎绫诲垽锟?(鏃х増 v6, 宸插簾锟?
//  * 鈹€鈹€ 搴熷純鍘熷洜: 鏂扮櫧璞嗘暟锟?R-B 澶ч潰绉浆锟?鏈€锟?723), 鏃ч槇鍊煎叏閮ㄥけ锟?鈹€鈹€
//  */
// uint8_t App_Rgb_Get_Result(void)
// {
//     uint16_t c = g_app_rgb_data.clean.clear;
//     uint16_t r = g_app_rgb_data.clean.red;
//     uint16_t g = g_app_rgb_data.clean.green;
//     uint16_t b = g_app_rgb_data.clean.blue;
//
//     int16_t gb = (int16_t)(g - b);  /* G-B */
//     int16_t rb = (int16_t)(r - b);  /* R-B */
//     int16_t rg = (int16_t)(r - g);  /* R-G */
//
//     if (c < 600)
//         return BEAN_NONE;
//
//     if (rb <= -100)
//        return BEAN_WHITE;
//     if (rg <= -1800)
//        return BEAN_WHITE;
//
//     if (rg <= -600)
//         return BEAN_YELLOW;
//     if (gb >= 750)
//         return BEAN_YELLOW;
//     if (rb > 300)
//         return BEAN_YELLOW;
//
//     if (rg >= -550 && gb <= 750)
//         return BEAN_GREEN;
//
//     return BEAN_NONE;
// }

// /**
//  * @brief 鏍规嵁 B/G 姣斿€艰繘琛岃眴瀛愬垎绫诲垽锟?(v7, 宸插簾锟?
//  * 鈹€鈹€ 搴熷純鍘熷洜: B/G 涓€缁寸壒寰佹棤娉曞悓鏃跺垏寮€缁胯眴/榛勮眴鍜岀櫧锟?榛勮眴涓ゅ閲嶅彔 鈹€鈹€
//  */
// uint8_t App_Rgb_Get_Result(void)
// {
//     uint16_t c = g_app_rgb_data.clean.clear;
//     uint16_t r = g_app_rgb_data.clean.red;
//     uint16_t g = g_app_rgb_data.clean.green;
//     uint16_t b = g_app_rgb_data.clean.blue;
//     int16_t rb = (int16_t)(r - b);
//     int16_t rg = (int16_t)(r - g);
//     if (c < 600) return BEAN_NONE;
//     uint8_t bg = (uint8_t)((b * 100) / g);
//     if (rb < 0) return BEAN_WHITE;
//     if (bg <= 64) return BEAN_GREEN;
//     if (bg >= 72) return BEAN_WHITE;
//     if (rg < 0) return BEAN_WHITE;
//     return BEAN_YELLOW;
// }

// /* ================================================================= */
// /* 5. 鏈€灏忎簩涔樼疮鍔犲櫒 & 2D 鏈€杩戦偦鍒嗙被鍣?(v8, 宸插簾寮?                 */
// /*   搴熷純鍘熷洜: 绱姞鍣ㄩ渶瑕佹敀 8 甯ф墠棣栨杈撳嚭, 寤惰繜 560ms 澶暱          */
// /* ================================================================= */
// typedef struct { uint64_t sum_g2, sum_rg, sum_bg; uint16_t count; uint8_t ready; } LsAccum_t;
// static LsAccum_t g_ls = {0};
// #define LS_MIN_SAMPLES 8
// static void App_Ls_Accumulate(void) { }  /* 瑙?v8 瀹屾暣瀹炵幇 */
// uint8_t App_Rgb_Get_Result(void) { }     /* 瑙?v8 瀹屾暣瀹炵幇 */

/* ================================================================= */
/* 5. alpha/beta 婊戝姩绐楀彛 & 2D 鏈€杩戦偦鍒嗙被鍣?(v9, 褰撳墠鐗堟湰)           */
/* ================================================================= */

/**
 * @brief alpha/beta 婊戝姩绐楀彛缂撳啿
 *
 * 姣?tick 璁＄畻鐬椂 alpha = R/G, beta = B/G (x1000), 鎺ㄥ叆 5 鏍锋湰鐜舰缂撳啿銆? * 鍙?5 甯х畻鏈钩鍧? 鏃㈡姉鍣０鍙堜笉寮曞叆绱姞鍣ㄥ欢杩熴€? * C < 600 鏃惰嚜鍔ㄦ竻闆? 鎹㈣眴鍚?100ms 鏀舵暃銆? */
typedef struct
{
    uint16_t buf_a[5];   /* alpha = R/G x 1000 */
    uint16_t buf_b[5];   /* beta  = B/G x 1000 */
    uint8_t  ptr;        /* current write pos  */
    uint8_t  count;      /* filled samples     */
} SlidingAB_t;

static SlidingAB_t g_ab = {0};
static uint16_t g_alpha = 0;   /* current valid alpha */
static uint16_t g_beta  = 0;   /* current valid beta  */

/**
 * @brief called each sampling cycle, updates alpha/beta sliding window
 */
static void App_SlidingAlphaBeta(void)
{
    uint16_t r = g_app_rgb_data.clean.red;
    uint16_t g = g_app_rgb_data.clean.green;
    uint16_t b = g_app_rgb_data.clean.blue;
    uint16_t c = g_app_rgb_data.clean.clear;
    uint8_t i;
    uint32_t sum_a, sum_b;

    /* no bean or weak signal -> reset window */
    if (c < 600)
    {
        for (i = 0; i < 5; i++) { g_ab.buf_a[i] = 0; g_ab.buf_b[i] = 0; }
        g_ab.ptr   = 0;
        g_ab.count = 0;
        g_alpha    = 0;
        g_beta     = 0;
        return;
    }

    /* instantaneous alpha = R/G, beta = B/G (x1000) */
    {
        uint16_t a = (uint16_t)(((uint32_t)r * 1000) / g);
        uint16_t bv = (uint16_t)(((uint32_t)b * 1000) / g);

        /* push to ring buffer */
        g_ab.buf_a[g_ab.ptr] = a;
        g_ab.buf_b[g_ab.ptr] = bv;
    }
    g_ab.ptr++;
    if (g_ab.ptr >= 5) g_ab.ptr = 0;
    if (g_ab.count < 5) g_ab.count++;

    /* compute window average */
    sum_a = 0; sum_b = 0;
    for (i = 0; i < g_ab.count; i++)
    {
        sum_a += g_ab.buf_a[i];
        sum_b += g_ab.buf_b[i];
    }
    g_alpha = (uint16_t)(sum_a / g_ab.count);
    g_beta  = (uint16_t)(sum_b / g_ab.count);
}

/**
 * @brief alpha/beta sliding window + 2D nearest-neighbor (v9)
 * @return 0:NONE  1:YELLOW  2:GREEN  3:WHITE
 *
 * cluster centers (from 2026-07-31 CSV least-squares regression):
 *   GREEN:  ( 913, 532 )
 *   YELLOW: ( 916, 661 )
 *   WHITE:  ( 765, 813 )
 */
uint8_t App_Rgb_Get_Result(void)
{
    uint16_t alpha, beta;
    int32_t da, db;
    uint32_t d_green, d_yellow, d_white;

    /* window not full yet */
    if (g_ab.count < 5)
        return BEAN_NONE;

    alpha = g_alpha;
    beta  = g_beta;

    /* to GREEN (913, 532) */
    da = (int32_t)alpha - 913;
    db = (int32_t)beta  - 532;
    d_green = (uint32_t)(da * da + db * db);

    /* to YELLOW (916, 661) */
    da = (int32_t)alpha - 916;
    db = (int32_t)beta  - 661;
    d_yellow = (uint32_t)(da * da + db * db);

    /* to WHITE (765, 813) */
    da = (int32_t)alpha - 765;
    db = (int32_t)beta  - 813;
    d_white = (uint32_t)(da * da + db * db);

    if (d_green < d_yellow && d_green < d_white)
        return BEAN_GREEN;
    if (d_yellow < d_white)
        return BEAN_YELLOW;
    return BEAN_WHITE;
}

/**
 * @brief 甯︾疆淇″害鏍￠獙鐨勭粨鏋滆緭鍑哄嚱锟?
 * @return 0:鏃犳晥/绛夊緟, 1-3:纭畾鐨勮眴瀛怚D
 */
uint8_t App_Rgb_Get_Validated_Result(void)
{
    static uint8_t last_raw_id = 0;     // 涓婁竴娆＄殑鐬椂璇嗗埆缁撴灉
    static uint16_t confidence_cnt = 0; // 缃俊搴﹁鏁板櫒
    static uint8_t final_stable_id = 0; // 鏈€缁堥攣瀹氱殑ID

    // 1. 鑾峰彇褰撳墠鐬椂鐨勮瘑鍒粨锟?(閫昏緫 C)
    uint8_t current_raw_id = App_Rgb_Get_Result();

    // 2. 濡傛灉褰撳墠璇嗗埆涓虹┖ (娌℃斁璞嗗瓙)锛屽垯閲嶇疆涓€鍒囩姸锟?
    if (current_raw_id == BEAN_NONE)
    {
        last_raw_id = BEAN_NONE;
        confidence_cnt = 0;
        final_stable_id = BEAN_NONE;
        return BEAN_NONE;
    }

    // 3. 濡傛灉褰撳墠璇嗗埆涓庝笂涓€娆′竴锟?
    if (current_raw_id == last_raw_id)
    {
        if (confidence_cnt < 20)
        { // 闃堝€艰瀹氾細杩炵画 20 娆′竴锟?(锟?200ms)
            confidence_cnt++;
        }
        else
        {
            // 杈惧埌缃俊搴﹁姹傦紝閿佸畾鏈€缁堣緭锟?
            final_stable_id = current_raw_id;
        }
    }
    // 4. 濡傛灉褰撳墠璇嗗埆涓庝笂涓€娆′笉涓€锟?(璇存槑鏁版嵁杩樺湪璺冲姩鎴栨崲浜嗚眴锟?
    else
    {
        confidence_cnt = 0; // 璁℃暟鍣ㄦ竻闆讹紝閲嶆柊寮€濮嬭锟?
        last_raw_id = current_raw_id;
        // 娉ㄦ剰锛氳繖閲屼笉绔嬪埢娓呯┖ final_stable_id锛岄槻姝㈢煭鏆傛瘺鍒哄鑷磋緭鍑轰腑锟?
    }

    // 5. 杩斿洖缁撴灉锛氬彧鏈夎揪鍒扮疆淇″害鎵嶈繑鍥炵湡锟?ID锛屽惁鍒欒繑鍥炵瓑寰呯姸锟?
    return (confidence_cnt >= 20) ? final_stable_id : BEAN_STABLE_WAITING;
}

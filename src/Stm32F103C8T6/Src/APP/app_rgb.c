#include "app_rgb.h"
#include <string.h>

static ColorRawData_t g_color_raw;
static ColorResult_t  g_color_result;

void App_Rgb_Init(void) {
    memset(&g_color_raw, 0, sizeof(g_color_raw));
    memset(&g_color_result, 0, sizeof(g_color_result));
}

void App_Rgb_GetRaw(ColorRawData_t *raw) {
    if (raw != NULL) {
        *raw = g_color_raw;
    }
}

void App_Rgb_GetResult(ColorResult_t *res) {
    if (res != NULL) {
        *res = g_color_result;
    }
}

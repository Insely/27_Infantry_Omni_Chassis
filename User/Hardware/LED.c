/*
 * @Date: 2025-08-31 21:36:57
 * @LastEditors: hao && (hao@qlu.edu.cn)
 * @LastEditTime: 2025-10-30 20:48:50
 * @FilePath: \Season-26-Code\User\Hardware\LED.c
 */
#include "LED.h"
#include "ws2812.h"

void HexToRGB(uint32_t hex, uint8_t *r, uint8_t *g, uint8_t *b)
{
    *r = (hex >> 16) & 0xFF; // 提取红色分量
    *g = (hex >> 8) & 0xFF;  // 提取绿色分量
    *b = hex & 0xFF;         // 提取蓝色分量
}

/**
 * @brief LED灯显示颜色
 *
 * @param color 颜色的rgb编码
 */
void LED_ShowColor(uint32_t color)
{
    uint8_t r, g, b;
    HexToRGB(color, &r, &g, &b);
    WS2812_Ctrl(r, g, b);
}

//void LEDchangecolor(uint32_t)
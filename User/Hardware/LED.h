#ifndef __LED_H__
#define __LED_H__
#include "stdint.h"


typedef enum {
    RED     = 0xFF0000,  // 红色
    GREEN   = 0x00FF00,  // 绿色
    BLUE    = 0x0000FF,  // 蓝色
    YELLOW  = 0xFFFF00,  // 黄色
    CYAN    = 0x00FFFF,  // 青色
    MAGENTA = 0xFF00FF,  // 品红
    BLACK   = 0x000000,  // 黑色
    WHITE   = 0xFFFFFF   // 白色
} Color;


void LED_ShowColor(uint32_t color);

#endif // !__LED_H__

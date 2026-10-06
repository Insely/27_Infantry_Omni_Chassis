/*
 * @Date: 2025-08-31 21:36:57
 * @LastEditors: hao && (hao@qlu.edu.cn)
 * @LastEditTime: 2025-10-30 20:15:19
 * @FilePath: \Season-26-Code\User\BSP\Stm32_time.c
 */
/**
 * @file time.c
 * @author sethome
 * @brief STM32F4的开机时间计算
 * @version 0.1
 * @date 2022-11-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "Stm32_time.h"
#include "stm32h7xx_hal.h"

uint32_t Get_SysTime_ms(void)
{
	return HAL_GetTick();
}

float Get_SysTime_s(void)
{
	return HAL_GetTick() / 1000.0f;
}

uint32_t cnt = 0;
void TIM_Count_100kHz(void)
{
	cnt+=10;
}
uint32_t Get_SysTime_us(void)
{
	return cnt;
}

//end of file

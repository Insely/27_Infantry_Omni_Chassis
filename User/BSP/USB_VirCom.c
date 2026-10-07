/*
 * @Date: 2025-08-31 21:36:57
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2025-12-24 00:06:06
 * @FilePath: \Reserve_Sentry\User\BSP\USB_VirCom.c
 */
/**
 * @file USB_VirCom.c
 * @author sethome
 * @brief 虚拟串口数据发送
 * @version 0.1
 * @date 2022-11-20
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "usbd_cdc_if.h"
#include "USB_VirCom.h"
#include "app_api.h"
#include <stdio.h>

void Vircom_Send(uint8_t data[], uint16_t len)
{
  CDC_Transmit_HS(data, len);
}

void Vircom_Rev(uint8_t data[], uint16_t len)
{
  App_OnUsbFrame(data, len);
}

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
  Vircom_Send((uint8_t *)&ch, 1);

  return ch;
}
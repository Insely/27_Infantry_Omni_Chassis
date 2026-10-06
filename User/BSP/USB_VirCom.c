/*
 * @Date: 2025-08-31 21:36:57
 * @LastEditors: hao && (hao@qlu.edu.cn)
 * @LastEditTime: 2025-10-30 20:22:21
 * @FilePath: \Season-26-Code\User\BSP\USB_VirCom.c
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
#include "CRC8_CRC16.h"
#include "Stm32_time.h"
#include "fifo.h"

#include "Global_status.h"
#include "Auto_control.h"



void Vircom_Send(uint8_t data[], uint16_t len)
{
  // if (CDC_Transmit_HS(data, len) == 1) // 判断数据是否发送
  // {
  //   // USB忙碌数据转入缓冲区

  //   fifo_s_puts(&USB_send_fifo, (char *)data, (int)len);
  // }
  CDC_Transmit_HS(data, len);

}

void Vircom_Rev(uint8_t data[], uint16_t len)
{

}

#include "stdio.h"
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
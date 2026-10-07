/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2026-01-05 22:25:36
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-05-12 08:35:24
 * @FilePath: \Season26_Regular_Sentry_Chassis\User\Hardware\vofa+.c
 */
#include "vofa+.h"
#include "stdint.h"
#include "UART_data_txrx.h"
#include "string.h"
#include "stdio.h"
#include "stdarg.h"

char tempData[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x00,0x00,0x80,0x7F};//前十6个是数据帧

void VOFA_UploadData(float data1, float data2, float data3, float data4)
{
    // 定义一个 20 字节的局部缓冲区（或者静态的）
    static uint8_t send_buf[20];
    float temp[4] = {data1, data2, data3, data4};
    
    // 1. 装载 16 字节的数据
    memcpy(send_buf, temp, 16);
    
    // 2. 显式装载 4 字节帧尾 (防止被意外修改)
    send_buf[16] = 0x00;
    send_buf[17] = 0x00;
    send_buf[18] = 0x80;
    send_buf[19] = 0x7F;

    // 3. 发送
    UART_SendData(UART10_data, send_buf, 20);
}

void VOFA_Printf(const char *fmt, ...)
{
	static uint8_t tx_buf[256] = {0};
	static va_list ap;
	static uint16_t len;
	va_start(ap,fmt);
	len = vsprintf((char*)tx_buf,fmt,ap);
	va_end(ap);
	//UART_send_data(UART1_data, tx_buf,len);
	
}

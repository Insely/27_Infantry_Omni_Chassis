#ifndef __FSI6X_H__
#define __FSI6X_H__

#include "stdint.h"

typedef struct 
{
    uint16_t CH1;//通道1数值
    uint16_t CH2;//通道2数值
    uint16_t CH3;//通道3数值
	uint16_t CH4;//通道4数值
	uint16_t CH5;//通道5数值
	uint16_t CH6;//通道6数值
	uint16_t CH7;//通道7数值
	uint16_t CH8;//通道8数值
	uint16_t CH9;//通道9数值
	uint16_t CH10;//通道10数值
	uint16_t CH11;//通道11数值
	uint16_t CH12;//通道12数值
	uint16_t CH13;//通道13数值
	uint16_t CH14;//通道14数值
	uint16_t CH15;//通道15数值
	uint16_t CH16;//通道16数值
	uint8_t ConnectState;//遥控器与接收器连接状态 0=未连接，1=正常连接
    int online;
}FSI6X_data_Struct;
extern FSI6X_data_Struct FSI6X_data;

#define FS_UP ((uint16_t)240)
#define FS_DOWN ((uint16_t)1807)
#define FS_MID ((uint16_t)1024)
#define NULL ((void*)0)

void FSI6X_decode_data(uint8_t *sbus_buf,FSI6X_data_Struct *SBUS_CH);

#endif 
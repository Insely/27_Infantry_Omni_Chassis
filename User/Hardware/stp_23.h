/* stp_23.h */
#ifndef __STP_23_H__
#define __STP_23_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h" 
#include <stdint.h>

/* 协议常量定义 */
#define STP23_HEADER         0x54
#define STP23_VER_LEN        0x2C
#define STP23_POINT_PER_PACK 12

/* --- 基础协议数据结构 --- */
typedef struct __attribute__((packed)) 
{
    uint16_t distance;  // 距离值 (mm)
    uint8_t  intensity; // 信号强度
} LidarPointDef;

typedef struct __attribute__((packed)) 
{
    uint8_t  header;        // 0x54
    uint8_t  ver_len;       // 0x2C
    uint16_t temperature;   // 原始温度值
    uint16_t start_angle;   // 起始角度
    LidarPointDef point[STP23_POINT_PER_PACK]; // 12个测量点
    uint16_t end_angle;     // 结束角度
    uint16_t timestamp;     // 时间戳
    uint8_t  crc8;          // 校验位
} LiDARFrameTypeDef;

/* --- 【新增】调试监控全家桶结构体 --- */
typedef struct 
{
    // 1. 最终结果区 (最常看的数据)
    uint16_t Distance_mm;      // 经过平滑处理后的当前距离
    uint16_t Temperature_Raw;  // 当前雷达温度 (原始ADC值)
    
    // 2. 原始帧数据区 (用于检查协议解析是否正确)
    LiDARFrameTypeDef LastFrame; // 最新接收到的完整一帧数据
    
    // 3. 统计诊断区 (用于检查通信质量)
    uint32_t Rx_Frame_Count;   // 成功接收并校验通过的帧数计数器 (一直在跳动说明正常)
    uint32_t Crc_Error_Count;  // 校验失败计数器 (如果不为0且在增加，说明干扰大或波特率不对)
    
} STP23_Monitor_t;

/* --- 全局变量声明 --- */
// 在任何地方都可以通过 extern 访问这个变量
extern STP23_Monitor_t STP23_Data;

/* --- 函数接口 --- */
void STP23_ProcessByte(uint8_t byte);

#ifdef __cplusplus
}
#endif

#endif /* __STP_23_H__ */
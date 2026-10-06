/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2026-01-09 04:13:14
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-01-09 04:13:25
 * @FilePath: \Regular_Sentry_Gimbal\User\Hardware\IMU\hi14.h
 */
#ifndef __HI14_H__

#ifdef __cplusplus
extern "C" {
#endif

#define __HI14_H__

#include "stdint.h"
#include "fdcan.h"
#include "CAN_receive_send.h"

typedef PACKED_STRUCT()
{
    struct
    {
        float HI14_Ax;
        float HI14_Ay;
        float HI14_Az;
    }acceleration;//加速度（g）
    struct
    {
        float HI14_Dx;
        float HI14_Dy;
        float HI14_Dz;
    }angular_velocity;//角速度（度每秒）
    struct
    {   
        float HI14_yaw;
        float HI14_pitch;
        float HI14_roll;
    } Cape_Euler;//欧拉角（度）quaternion
    struct
    {
        float HI14_qw;
        float HI14_qx;
        float HI14_qy;
        float HI14_qz;
    } quaternion;//四元数
    float atmospheric_pressure;//气压（pa）
     struct
    {
        float HI14_xAngle;
        float HI14_yAngle;
    }inclination;//倾角（度）
}
HI14_data_t;

typedef enum
{
    HI14_send_id = 0x600,
    HI14_ID = 0X01, // 如果想要修改ID，先调用修改ID的函数，烧录成功后再修改这里的ID
} HI14_canid;
typedef enum
{
    HI14_0_HZ,
    HI14_10_HZ,
    HI14_20_HZ,
    HI14_50_HZ,
    HI14_100_HZ,
    HI14_200_HZ,
} HI14_Frequency;
typedef enum
{
   Cceleration,//加速度
   Angular_velocity,//角速度
   Cape_Euler,//欧拉角
   Quaternion,//四元数
   Atmospheric_pressure,//气压
} HI14_parameter;

extern HI14_data_t hi14;
//外部调用
void HI14_ID_Change(uint8_t ID, uint8_t target_ID);
void HI14_frequency_change(uint8_t ID, uint8_t parameter, uint8_t frequency);

#ifdef __cplusplus
}
#endif
#endif // !__HI14_H__
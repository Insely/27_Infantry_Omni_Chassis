#ifndef __AUTO_CONTROL_H__
#define __AUTO_CONTROL_H__

#include "stdint.h"

#pragma pack(1)

/**
 * @brief 机器人运行模式 (下位机 -> 上位机)
 * @author Nas(1319621819@qq.ocm)
 */
typedef enum {
    MODE_IDLE       = 0, // 空闲
    MODE_AUTO_AIM   = 1, // 自瞄
    MODE_SMALL_BUFF = 2, // 小符
    MODE_BIG_BUFF   = 3, // 大符
    MODE_OUTPOST    = 4  // 前哨站 (保留)
} RobotMode_e;

/**
 * @brief 视觉控制模式 (上位机 -> 下位机)
 * @author Nas(1319621819@qq.ocm)
 */
typedef enum {
    CTRL_NO_CONTROL     = 0, // 不控制
    CTRL_AIM_ONLY       = 1, // 控制云台但不开火
    CTRL_AIM_AND_FIRE   = 2  // 控制云台且开火
} VisionControlMode_e;

/**
 * @brief 上位机发送给STM32的数据 (MiniPC -> STM32)
 * @author Nas(1319621819@qq.ocm)
 */
typedef struct {
    uint8_t header[2] ;   // 帧头 (固定值 {'S', 'P'})

    uint8_t mode;        // 控制模式 (VisionControlMode_e)

    float yaw;           // 偏航角 (rad)
    float yaw_vel;       // 偏航角速度 (rad/s)
    float yaw_acc;       // 偏航角加速度 (rad/s^2)

    float pitch;         // 俯仰角 (rad)
    float pitch_vel;     // 俯仰角速度 (rad/s)
    float pitch_acc;     // 俯仰角加速度 (rad/s^2) 

    float center_yaw;    // yaw轴旋转中心
    //float distance;      // 目标到相机距离

    uint16_t crc16;      // CRC16 校验
} MINIPC_data_t;

/**
 * @brief STM32发送给上位机的数据 (STM32 -> MiniPC)
 * @author Nas(1319621819@qq.ocm)
 */
typedef struct {
    uint8_t header[2];   // 帧头 (固定值 {'S', 'P'})
  
    uint8_t mode;        // 当前模式 (RobotMode_e)

    // --- IMU 姿态 (四元数) ---
    float q[4];          // w, x, y, z 顺序

    // --- 云台状态 ---
    float yaw;           // 当前偏航角 (rad)
    float yaw_vel;       // 当前偏航角速度 (rad/s)
    float pitch;         // 当前俯仰角 (rad)
    float pitch_vel;     // 当前俯仰角速度 (rad/s)

    // --- 射击状态 ---
    float bullet_speed;      // 弹速 (m/s)
    uint16_t bullet_count;   // 子弹累计发送次数 (用于统计/丢包检测)

    uint16_t crc16;      // CRC16 校验
}   STM32_data_t;

#pragma pack(4)

void STM32_to_MINIPC();
void decodeMINIPCdata(MINIPC_data_t *target, unsigned char buff[], unsigned int len);
void Auto_Control();
void MINIPC_to_STM32();

extern MINIPC_data_t fromMINIPC;
extern STM32_data_t toMINIPC;

#endif
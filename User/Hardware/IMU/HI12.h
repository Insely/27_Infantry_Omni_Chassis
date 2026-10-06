/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2026-01-27 16:02:56
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-01-27 18:18:50
 * @FilePath: \Regular_Sentry_Gimbal\User\Hardware\IMU\HI12.h
 */
#ifndef __HI12_H__

#ifdef __cplusplus
extern "C" {
#endif

#define __HI12_H__

#include "stdint.h"
#include "fdcan.h"
#include "CAN_receive_send.h"
#include "stm32h7xx_hal.h"

#define PGN_UTC_TIME      0xFF2F  // 时间信息 
#define PGN_ACCEL         0xFF34  // 三轴加速度 
#define PGN_GYRO          0xFF37  // 三轴角速度 
#define PGN_EULER_RP      0xFF3D  // 俯仰横滚角 
#define PGN_EULER_YAW     0xFF41  // 航向角 ---bytes[0]~byte[3]是0-360°表示， 顺时针为正
                                  //        ---bytes[4]~byte[7]是±180°表示， 逆时针为正
#define PGN_MAGNETOMETER  0xFF3A  // 地磁计                                        
#define PGN_QUATERNION    0xFF46  // 四元数 
#define PGN_INCLINOMETER  0xFF4A  // 倾角仪  ---bytes[0]~byte[3]是X轴倾角， 可配置0-360°或±180°
                                  //         ---bytes[4]~byte[7]Y轴倾角， 可配置0-360°或±90°

#include "hipnuc_can_driver.h"

extern can_sensor_data_t HI12_Data;

void HI12_Init(FDCAN_HandleTypeDef *hcan);
/* int Parse_Frame(const hipnuc_can_frame_t *frame, can_sensor_data_t *data); */
int Parse_Frame(uint32_t raw_id,uint8_t *data_ptr, can_sensor_data_t *data);

void HI12_Decode(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);
                                  

#ifdef __cplusplus
}
#endif
#endif

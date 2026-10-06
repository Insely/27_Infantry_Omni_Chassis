/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2026-01-04 04:39:33
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-01-04 04:41:15
 * @FilePath: \Regular_Sentry_Gimbal\User\Hardware\IMU\xrobot_imu.h
 */

#ifndef __XROBOT_IMU_H__

#ifdef __cplusplus
extern "C" {
#endif

#define __XROBOT_IMU_H__

#include"stdint.h"

typedef struct __attribute__((packed)) {
  float x;
  float y;
  float z;
} Vector3;

typedef struct __attribute__((packed)) {
  float q0;
  float q1;
  float q2;
  float q3;
} Quaternion;

typedef struct __attribute__((packed)) {
  float yaw;
  float pit;
  float rol;
} EulerAngles;

typedef struct __attribute__((packed)) {
  uint64_t time : 48;
  uint64_t sync : 48;
  Quaternion quat_;
  Vector3 gyro_;
  Vector3 accl_;
  EulerAngles eulr_;
} Data;

#define IMU_DEVICE_ID (0x30)
#define M_PI 3.14159265358979323846
#define M_2PI 6.28318530717958647692
/* CAN Packet IDs */
#define CAN_PACK_ID_ACCL 0
#define CAN_PACK_ID_GYRO 1
#define CAN_PACK_ID_EULR 3
#define CAN_PACK_ID_QUAT 4
/* Encoder constants */
#define ENCODER_21_MAX_INT ((1u << 21) - 1)

#ifdef __cplusplus
}
#endif
#endif

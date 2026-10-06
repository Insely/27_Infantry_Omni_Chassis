/*
 * @Author: hao hao@qlu.edu.cn
 * @Date: 2025-08-31 21:36:57
 * @LastEditors: hao && (hao@qlu.edu.cn)
 * @LastEditTime: 2025-10-03 10:06:39
 * @FilePath: \Season-26-Code\User\BSP\CAN_receive_send.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __CAN_RECEIVE_SEND_H__
#define __CAN_RECEIVE_SEND_H__

//#include "cover_headerfile_h.h"
#include "fdcan.h"

/* 所有 Yaw 诊断集中在一个变量中，调试器只需添加 yaw_debug。 */
typedef struct
{
  struct
  {
    uint32_t rx_0x102_max_interval_ms;
    uint32_t rx_lost_or_full_count;
    uint32_t tx_fail_count;
  } can2;

  struct
  {
    uint32_t feedback_max_interval_ms;
    uint32_t feedback_interval_ms;
    uint32_t feedback_gap_over_10ms_count;
    uint32_t feedback_last_gap_interval_ms;
    uint32_t feedback_last_gap_tick_ms;
    uint32_t rx_lost_or_full_count;
    uint32_t tx_fail_count;
    uint32_t error_recovery_count;
    uint32_t tx_error_count;
    uint32_t rx_error_count;
    uint32_t error_logging_count;
    uint32_t error_status_count;
    uint32_t last_error_status_its;
    uint32_t last_error_code;
    uint32_t error_status_last_tick_ms;
  } can3;

  struct
  {
    uint32_t non1_feedback_count;
    uint32_t non1_event_count;
    uint32_t last_non1_tick_ms;
    int32_t last_non1_state;
    float last_non1_tmos;
    float last_non1_tcoil;
    float last_non1_torque;
    float last_non1_velocity;
    float last_non1_velocity_set;
    float max_tmos;
    float max_tcoil;
    float max_abs_torque;
    float max_abs_velocity_error;
    uint32_t max_abs_velocity_error_tick_ms;
    uint32_t tx_call_interval_ms;
    uint32_t tx_call_max_interval_ms;
    uint32_t tx_call_gap_over_10ms_count;
    uint32_t tx_call_last_gap_tick_ms;
    uint32_t tx_call_last_gap_interval_ms;
  } motor;

  struct
  {
    uint32_t gimbal_interval_ms;
    uint32_t gimbal_max_interval_ms;
    uint32_t gimbal_gap_over_10ms_count;
    uint32_t gimbal_last_gap_tick_ms;
    uint32_t gimbal_last_gap_interval_ms;
  } task;

  struct
  {
    uint8_t freeze_active;
    uint32_t freeze_event_count;
    uint32_t same_value_duration_ms;
    uint32_t same_value_max_duration_ms;
    uint32_t freeze_last_start_tick_ms;
    uint32_t freeze_last_detect_tick_ms;
    uint32_t freeze_last_end_tick_ms;
    uint32_t freeze_last_duration_ms;
    float freeze_last_cmd;
    float freeze_last_start_yaw_cnt;
    float freeze_last_detect_yaw_cnt;
    float freeze_last_end_yaw_cnt;
    float freeze_last_actual_velocity;
    float freeze_last_torque;
    float freeze_last_imu_gyro_z;
    float freeze_last_body_gyro_z;
    uint32_t freeze_last_motor_state;
  } command;
} YawDebug_t;

extern volatile YawDebug_t yaw_debug;

extern FDCAN_HandleTypeDef* Get_CanHandle(uint8_t can_bus);

extern void Can_Init(void);

extern uint8_t Fdcanx_SendData(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
extern uint8_t Fdcanx_Receive(FDCAN_HandleTypeDef *hfdcan,	FDCAN_RxHeaderTypeDef *fdcan_RxHeader, uint8_t *buf);

extern void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan);
extern void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs);


#endif /* __CAN_RECEIVE_SEND_H__ */


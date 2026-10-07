/**
 * @file  app_api.h
 * @brief FreeRTOS 任务、硬件回调与步兵应用层之间的统一接口。
 */

#ifndef INCLUDED_APP_API_H
#define INCLUDED_APP_API_H

#include <stdint.h>

/* 应用公共状态及双板通信初始化；在启动调度器前调用。 */
void App_Init(void);

/* 三个执行机构任务各自的初始化入口。 */
void App_GimbalInit(void);
void App_ChassisInit(void);
void App_ShootInit(void);

/* 与 .ioc 中七个 FreeRTOS 任务对应的周期入口。 */
void App_RemoteStep(void);
void App_GimbalStep(void);
void App_ChassisStep(void);
void App_MotorStep(void);
void App_ShootStep(void);
void App_RefereeStep(void);

/* 板间通信周期入口，由 Chassis 任务每 1 ms 调用。 */
void App_LinkStep(void);

/* 外部事件与硬件定时器入口。 */
void App_RequestUiReset(void);
void App_OnUsbFrame(const uint8_t *data, uint16_t len);
void App_OnVisionBytes(const uint8_t *data, uint16_t len);
void App_OnNavigationBytes(const uint8_t *data, uint16_t len);
void App_SendVisionTelemetry(void);
void App_ImuStep(void);

/* 返回 1 时允许 Motor_control 任务刷新独立看门狗。 */
uint8_t App_WatchdogRefreshAllowed(void);

#endif /* INCLUDED_APP_API_H */

/**
 * @file  Chassis.h
 * @brief 底盘板的全向轮底盘接口选择。
 */

#ifndef INCLUDED_CHASSIS_H
#define INCLUDED_CHASSIS_H

#include "robot_param.h"

#if (USE_CHASSIS_OMNI != 0)
#include "Chassis_omni.h"
#else
/* 未启用底盘时保留空接口，供公共任务代码调用。 */
static inline void Chassis_Init(void) {}
static inline void Chassis_Tasks(void) {}
static inline void Chassis_SetX(float x) { (void)x; }
static inline void Chassis_SetY(float y) { (void)y; }
static inline void Chassis_SetR(float r) { (void)r; }
static inline void Chassis_SetAccel(float acc) { (void)acc; }
#endif

#endif /* INCLUDED_CHASSIS_H */

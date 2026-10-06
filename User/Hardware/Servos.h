#ifndef __PWM_CONTROL_H__
#define __PWM_CONTROL_H__

#include "stdint.h"

#define PWM_PIN_1 1
#define PWM_PIN_2 2
#define PWM_PIN_3 3
#define PWM_PIN_4 4

void PWM_ControlInit(void);
void Set_ServoAngle(uint8_t channel, float angle); //统一给180舵机

#endif

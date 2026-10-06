#ifndef TEST_FDCAN_H
#define TEST_FDCAN_H
#include <stdint.h>
typedef struct { int bus; } FDCAN_HandleTypeDef;
extern FDCAN_HandleTypeDef hfdcan1,hfdcan2,hfdcan3;
uint32_t HAL_GetTick(void);
#endif

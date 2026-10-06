#include "fdcan.h"
FDCAN_HandleTypeDef *Get_CanHandle(uint8_t bus);
uint8_t Fdcanx_SendData(FDCAN_HandleTypeDef *,uint16_t,uint8_t *,uint32_t);

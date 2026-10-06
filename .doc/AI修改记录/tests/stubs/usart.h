#ifndef TEST_USART_H
#define TEST_USART_H
#include <stdint.h>
typedef struct { int port; } UART_HandleTypeDef;
extern UART_HandleTypeDef huart1,huart2,huart3;
#define HAL_OK 0
int HAL_UART_Transmit_DMA(UART_HandleTypeDef *,uint8_t *,uint16_t);
#endif

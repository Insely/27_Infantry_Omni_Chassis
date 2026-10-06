/**
 * @file UART_data_txrx.c
 * @author sethome
 * @brief 串口数据发送接受
 * @version 0.1
 * @date 2022-11-20
 *
 * @copyright Copyright (c) 2022 sethome
 *
 */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "fifo.h"

#include "UART_data_txrx.h"

#include "DT7.h"
#include "VT13.h"
#include "IMU_updata.h"
#include "referee_system.h"
#include "motor.h"

#include "Global_status.h"
#include "Auto_control.h"
#include "app_api.h"
#include "BoardLink.h"


// DMA控制变量
extern DMA_HandleTypeDef hdma_uart5_rx;    // 遥控器，仅用接受
extern DMA_HandleTypeDef hdma_uart8_rx;    // 扩展串口8，连接云台IMU
extern DMA_HandleTypeDef hdma_uart7_rx;    // 串口7，连接电源管理模块
extern DMA_HandleTypeDef hdma_uart7_tx;
extern DMA_HandleTypeDef hdma_usart10_rx;  // 串口10，连接图传模块
extern DMA_HandleTypeDef hdma_usart10_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;   //串口1，连接视觉小电脑
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

// 串口控制变量
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5; // 遥控器，可能用不到
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart10;

// 将上述串口+DMA整合，并包含缓冲区
transmit_data UART1_data;
transmit_data UART2_data;
transmit_data UART3_data;
transmit_data UART5_data;
transmit_data UART7_data;
transmit_data UART8_data;
transmit_data UART10_data;

/**
 * @brief 串口初始化
 *
 * @return * void
 */
void Uart_Init(void)
{
#if BOARD_LINK_TRANSPORT == BOARD_LINK_TRANSPORT_UART
  /* 普通 UART 板间通信使用 1 Mbaud；配置放在 BSP 层，避免修改 CubeMX 生成区。 */
  huart1.Init.BaudRate = 1000000U;
  if (HAL_UART_Init(&huart1) != HAL_OK)
    Error_Handler();
#endif

  Uart_DMARxTxStart(&UART1_data, &huart1, &hdma_usart1_rx, &hdma_usart1_tx);
  Uart_DMARxTxStart(&UART2_data, &huart2, &hdma_usart2_rx, &hdma_usart2_tx);
  Uart_DMARxTxStart(&UART3_data, &huart3, &hdma_usart3_rx, &hdma_usart3_tx);
  Uart_DMARxTxStart(&UART5_data, &huart5, &hdma_uart5_rx, &hdma_uart5_rx);
  Uart_DMARxTxStart(&UART7_data, &huart7, &hdma_uart7_rx, &hdma_uart7_tx);
  Uart_DMARxTxStart(&UART8_data, &huart8, &hdma_uart8_rx, &hdma_uart8_rx);
  Uart_DMARxTxStart(&UART10_data, &huart10, &hdma_usart10_rx, &hdma_usart10_tx);
}

/**
 * @brief DMA，串口中断启动，ヾ(?ω?`)o温馨提示，可以在头文件里用宏自定义串口缓冲区大小，
 * @param data 串口整合包指针
 * @param huart 串口指针
 * @param hdma_usart_rx 串口接受dma指针
 * @param hdma_usart_tx 串口发送dma指针
 */
void Uart_DMARxTxStart(transmit_data *data, UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx, DMA_HandleTypeDef *hdma_usart_tx)
{
  data->huart = huart;                 // 串口控制变量
  data->hdma_usart_rx = hdma_usart_rx; // DMA接收缓冲
  data->hdma_usart_tx = hdma_usart_tx; // DMA发送缓冲

  HAL_UARTEx_ReceiveToIdle_DMA(huart, data->rev_data, UART_BUFFER_SIZE); // 开启DMA批量数据接受
  __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);                        // 关闭接受过半中断
}

/**
 * @brief 串口发送数据
 *
 * @param uart 发送串口整合包
 * @param data 发送数据（数据别释放了，不然后面收不到）
 * @param size 数据大小
 */
void UART_SendData(transmit_data uart, uint8_t data[], uint16_t size)
{
  //+++++++++++++++//while(HAL_DMA_GetState(UART6_data.hdma_usart_tx) != HAL_DMA_STATE_READY)
  HAL_UART_Transmit_DMA(uart.huart, data, size); // 套娃ヾ(?ω?`)o
}

/**
?* @brief 串口接受空回调函数，用于接受不定长数据，放置数据处理函数
?* @note ?该函数为HAL库中断函数，无需在主函数中调用
?* @param huart 发生中断的串口句柄
?* @param Size ?接收到的数据长度
?*/
static transmit_data *Uart_Context(UART_HandleTypeDef *uart)
{
    if (uart==&huart1) return &UART1_data;
    if (uart==&huart2) return &UART2_data;
    if (uart==&huart3) return &UART3_data;
    if (uart==&huart5) return &UART5_data;
    if (uart==&huart7) return &UART7_data;
    if (uart==&huart8) return &UART8_data;
    if (uart==&huart10) return &UART10_data;
    return NULL;
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    transmit_data *ctx=Uart_Context(huart);
    if (!ctx || size>UART_BUFFER_SIZE) return;
    uint8_t *data=ctx->rev_data;
    if (!BoardLink_UartRxDispatch(huart,data,size)) {
#if BOARD_GIMBAL
        if (huart==&huart1) App_OnVisionBytes(data,size);
        else if (huart==&huart5) {
            if (size>=25 && data[0]==0x0F) FSI6X_decode_data(data,&FSI6X_data);
            else if (size==18) DT7_DecodeData(data);
        }
        else if (huart==&huart10 && size==21) VT13_DataSolve(data,&VT13_data);
#else
        if (huart==&huart7) fifo_s_puts(&referee_fifo,(char *)data,size);
#endif
    }
    HAL_UARTEx_ReceiveToIdle_DMA(huart,data,UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx,DMA_IT_HT);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) { (void)huart; }
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) { BoardLink_UartTxCpltCallback(huart); }
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    transmit_data *ctx=Uart_Context(huart);
    if (!ctx) return;
    HAL_UART_AbortTransmit(huart);
    BoardLink_UartErrorCallback(huart);
    HAL_UARTEx_ReceiveToIdle_DMA(huart,ctx->rev_data,UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx,DMA_IT_HT);
}

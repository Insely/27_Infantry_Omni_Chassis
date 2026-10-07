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
#include "FSI6X.h"
#include "VT13.h"
#include "IMU_updata.h"
#include "referee_system.h"
#include "motor.h"

#include "Global_status.h"
#include "Auto_control.h"
#include "app_api.h"
#include "BoardLink.h"
#include "stp_23.h"
#include "vofa+.h"
#include "supercup.h"


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
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (Size > UART_BUFFER_SIZE)
    return;

  // 使用 uintptr_t 将指针转换为整数类型，以便在 switch 中使用
  // huart->Instance 指向触发中断的硬件串口(如 USART1, UART5)
  switch ((uintptr_t)huart->Instance)
  {
  case (uintptr_t)USART1: // 双板普通 UART 通信 / 自瞄数据
  {
    if (!BoardLink_UartRxDispatch(huart, UART1_data.rev_data, Size))
    {
#if BOARD_GIMBAL
      App_OnVisionBytes(UART1_data.rev_data, Size);
#endif
    }

    // 重新启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(huart, UART1_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); // 关闭半传输中断
    break;
  }

  case (uintptr_t)USART2:
  {
    (void)BoardLink_UartRxDispatch(huart, UART2_data.rev_data, Size);

    // 重新启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(huart, UART2_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); // 关闭半传输中断
    break;
  }

  case (uintptr_t)USART3:
  {
    (void)BoardLink_UartRxDispatch(huart, UART3_data.rev_data, Size);

    // 重新启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(huart, UART3_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); // 关闭半传输中断
    break;
  }

  case (uintptr_t)UART5: // 遥控器
  {
    if (!BoardLink_UartRxDispatch(huart, UART5_data.rev_data, Size))
    {
      // 与原步兵工程相同：SBUS 帧头优先，其余按 DT7 解包。
      // 保留最小长度检查，防止不足 18 字节时读取缓冲区旧数据。
      if (Size >= 25 && UART5_data.rev_data[0] == 0x0F)
      {
        FSI6X_decode_data(UART5_data.rev_data, &FSI6X_data);
      }
      else if (Size >= RC_FRAME_LENGTH)
      {
        DT7_DecodeData(UART5_data.rev_data);
      }
    }

    // 重新启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(huart, UART5_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); // 关闭半传输中断
    break;
  }

  case (uintptr_t)UART7: // 导航 / 电管裁判系统
  {
    if (!BoardLink_UartRxDispatch(huart, UART7_data.rev_data, Size))
    {
#if BOARD_GIMBAL
      App_OnNavigationBytes(UART7_data.rev_data, Size);
#else
      fifo_s_puts(&referee_fifo, (char *)UART7_data.rev_data, (int)Size);
#endif
    }

    // 重新启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(huart, UART7_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); // 关闭半传输中断
    break;
  }

  case (uintptr_t)UART8: // 云台IMU
  {
    (void)BoardLink_UartRxDispatch(huart, UART8_data.rev_data, Size);

    // 重新启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(huart, UART8_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); // 关闭半传输中断
    break;
  }

  case (uintptr_t)USART10: // 图传链路裁判系统 -> STP-23协议
  {
    if (!BoardLink_UartRxDispatch(huart, UART10_data.rev_data, Size))
    {
      if (Size == 21 &&
          UART10_data.rev_data[0] == 0xA9 && UART10_data.rev_data[1] == 0x53)
      {
        VT13_DataSolve(UART10_data.rev_data, &VT13_data);
      }

#if BOARD_GIMBAL
      /* STP-23 是字节流协议。
         DMA接收到的是一串数据（Buffer），长度为 Size。
         我们需要遍历 Buffer，把每个字节传给状态机处理。
      */
      if (!(Size == 21 &&
            UART10_data.rev_data[0] == 0xA9 && UART10_data.rev_data[1] == 0x53))
      {
        for (uint16_t i = 0; i < Size; i++)
          STP23_ProcessByte(UART10_data.rev_data[i]);
      }
#else
      VOFA_UploadData(cap.Chassis_power, 0, 0, 0);
#endif
    }

    // 重新启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(huart, UART10_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); // 关闭半传输中断
    break;
  }

  default:
  {
    // 可选：处理未知的 huart 实例
    break;
  }
  }
}

/**
 * @brief 串口接受完成中断回调函数，用于接受定长数据
 *
 * @param huart 接受串口号
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  (void)huart;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  BoardLink_UartTxCpltCallback(huart);
}

// 发生错误重启串口
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_AbortTransmit(huart);
  BoardLink_UartErrorCallback(huart);

  switch ((uintptr_t)huart->Instance)
  {
  case (uintptr_t)USART1:
  {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART1_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    break;
  }

  case (uintptr_t)USART2:
  {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UART2_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    break;
  }

  case (uintptr_t)USART3:
  {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, UART3_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    break;
  }

  case (uintptr_t)UART5:
  {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, UART5_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    break;
  }

  case (uintptr_t)UART7:
  {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, UART7_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    break;
  }

  case (uintptr_t)UART8:
  {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart8, UART8_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    break;
  }

  case (uintptr_t)USART10:
  {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart10, UART10_data.rev_data, UART_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    break;
  }

  default:
  {
    break;
  }
  }
}
// end of file

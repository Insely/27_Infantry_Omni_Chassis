/**
 * @file CAN_receive_send.c
 * @author Siri (lixirui2017@outlook.com)
 * @brief can bsp层发送与接受
 * @version 0.1
 * @date 2024-10-19
 *
 * @copyright Copyright (c) 2024
 *
 */
// #include "cover_headerfile_h.h"
#include "CAN_receive_send.h"
#include "BoardLink.h"
#include "motor.h"
#include "dm_imu.h"
#include "supercup.h"

// CAN寄存器及控制器
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3; // 定义原型在fdcan.c文件

#define BOARD_LINK_CHASSIS_SPEED_W_ID 0x102U
#define YAW_MOTOR_FEEDBACK_ID          0x002U
#define YAW_FEEDBACK_GAP_THRESHOLD_MS  10U
#define CAN3_ERROR_STATUS_NOTIFICATIONS (FDCAN_IT_ERROR_WARNING | \
                                         FDCAN_IT_ERROR_PASSIVE | \
                                         FDCAN_IT_BUS_OFF)

volatile YawDebug_t yaw_debug = {0};

static uint32_t can2_0x102_last_rx_tick = 0U;
static uint8_t can2_0x102_seen = 0U;
static uint32_t can3_yaw_feedback_last_rx_tick = 0U;
static uint8_t can3_yaw_feedback_seen = 0U;


/**
 * @brief 获取指定CAN总线的句柄
 */
FDCAN_HandleTypeDef* Get_CanHandle(uint8_t can_bus) {
    switch (can_bus) {
        case 0: return &hfdcan1;
        case 1: return &hfdcan2;
        case 2: return &hfdcan3;
        default: return &hfdcan1;
    }
}

/**
 * @brief 初始化can,包含过滤器配置与使能
 *
 */
void Can_Init(void)
{
  FDCAN_FilterTypeDef fdcan_filter;

  yaw_debug = (YawDebug_t){0};
  can2_0x102_last_rx_tick = 0U;
  can2_0x102_seen = 0U;
  can3_yaw_feedback_last_rx_tick = 0U;
  can3_yaw_feedback_seen = 0U;

  fdcan_filter.IdType = FDCAN_STANDARD_ID;             // 过滤标准ID
  fdcan_filter.FilterIndex = 0;                        // 滤波器索引
  fdcan_filter.FilterType = FDCAN_FILTER_MASK;         // 掩码模式
  fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 过滤器0关联到FIFO0
  fdcan_filter.FilterID1 = 0x00000000;                 // 不去过滤任何ID
  fdcan_filter.FilterID2 = 0x00000000;                 // 同上

  HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter); // 将上述配置到CAN1
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_Start(&hfdcan1);

  HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filter);
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  HAL_FDCAN_ActivateNotification(&hfdcan2,
                                 FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                 FDCAN_IT_RX_FIFO0_FULL |
                                 FDCAN_IT_RX_FIFO0_MESSAGE_LOST,
                                 0);
  HAL_FDCAN_Start(&hfdcan2);

  HAL_FDCAN_ConfigFilter(&hfdcan3, &fdcan_filter);
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  HAL_FDCAN_ActivateNotification(&hfdcan3,
                                 FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                 FDCAN_IT_RX_FIFO0_FULL |
                                 FDCAN_IT_RX_FIFO0_MESSAGE_LOST |
                                 CAN3_ERROR_STATUS_NOTIFICATIONS,
                                 0);
  HAL_FDCAN_Start(&hfdcan3);
}

/**
************************************************************************
* @brief:      	Fdcanx_SendData(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan：FDCAN句柄
* @param:       id：CAN设备ID
* @param:       data：发送的数据
* @param:       len：发送的数据长度
* @retval:     	void
* @details:    	发送数据
************************************************************************
**/
uint8_t Fdcanx_SendData(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
  if (hfdcan == NULL || data == NULL || len > 8U || id > 0x7FFU) return 1;
  FDCAN_TxHeaderTypeDef TxHeader = {0};

  TxHeader.Identifier = id;
  TxHeader.IdType = FDCAN_STANDARD_ID;              // 标准ID
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;          // 数据帧
  static const uint32_t dlc[] = {FDCAN_DLC_BYTES_0,FDCAN_DLC_BYTES_1,FDCAN_DLC_BYTES_2,FDCAN_DLC_BYTES_3,FDCAN_DLC_BYTES_4,FDCAN_DLC_BYTES_5,FDCAN_DLC_BYTES_6,FDCAN_DLC_BYTES_7,FDCAN_DLC_BYTES_8};
  TxHeader.DataLength = dlc[len];          // 发送数据长度
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;  // 设置错误状态指示
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;           // 不开启可变波特率
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;            // 普通CAN格式
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 用于发送事件FIFO控制, 不存储
  TxHeader.MessageMarker = 0x00;                    // 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0到0xFF

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data) != HAL_OK)
  {
    if (hfdcan == &hfdcan2)
      yaw_debug.can2.tx_fail_count++;
    else if (hfdcan == &hfdcan3)
      yaw_debug.can3.tx_fail_count++;
    return 1; // 发送*
  }
  return 0;
}

/**
************************************************************************
* @brief:      	Fdcanx_Receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
* @param:       hfdcan：FDCAN句柄
* @param:       buf：接收数据缓存
* @retval:     	接收的数据长度
* @details:    	接收数据
************************************************************************
**/
uint8_t Fdcanx_Receive(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef *fdcan_RxHeader, uint8_t *buf)
{
  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, fdcan_RxHeader, buf) != HAL_OK)
    return 0; // 接收数据
  return fdcan_RxHeader->DataLength >> 16;
}

/**
 * @brief CAN接受回调函数
 *
 * @param hfdcan
 * @param RxFifo0ITs
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef rx_header; // CAN 数据指针
  uint8_t rx_data[64];              // 获取到的数据

  if (hfdcan == &hfdcan2 &&
      (RxFifo0ITs & (FDCAN_IT_RX_FIFO0_FULL |
                     FDCAN_IT_RX_FIFO0_MESSAGE_LOST)) != RESET)
  {
    yaw_debug.can2.rx_lost_or_full_count++;
  }
  else if (hfdcan == &hfdcan3 &&
           (RxFifo0ITs & (FDCAN_IT_RX_FIFO0_FULL |
                          FDCAN_IT_RX_FIFO0_MESSAGE_LOST)) != RESET)
  {
    yaw_debug.can3.rx_lost_or_full_count++;
  }

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == RESET &&
      HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) == 0U)
    return;

  /* 一次中断排空 FIFO，避免双板多帧突发时后续帧长期滞留或溢出。 */
  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
  {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
      break;

    if (rx_header.IdType != FDCAN_STANDARD_ID || rx_header.RxFrameType != FDCAN_DATA_FRAME ||
        rx_header.DataLength != FDCAN_DLC_BYTES_8) continue;

    if (hfdcan == &hfdcan2 && rx_header.Identifier == BOARD_LINK_CHASSIS_SPEED_W_ID)
    {
      uint32_t now = HAL_GetTick();

      if (can2_0x102_seen != 0U)
      {
        uint32_t interval = now - can2_0x102_last_rx_tick;
        if (interval > yaw_debug.can2.rx_0x102_max_interval_ms)
          yaw_debug.can2.rx_0x102_max_interval_ms = interval;
      }
      else
        can2_0x102_seen = 1U;
      can2_0x102_last_rx_tick = now;
    }

    if (hfdcan == &hfdcan3 && rx_header.Identifier == YAW_MOTOR_FEEDBACK_ID)
    {
      uint32_t now = HAL_GetTick();

      if (can3_yaw_feedback_seen != 0U)
      {
        uint32_t interval = now - can3_yaw_feedback_last_rx_tick;
        yaw_debug.can3.feedback_interval_ms = interval;
        if (interval > yaw_debug.can3.feedback_max_interval_ms)
          yaw_debug.can3.feedback_max_interval_ms = interval;
        if (interval > YAW_FEEDBACK_GAP_THRESHOLD_MS)
        {
          yaw_debug.can3.feedback_gap_over_10ms_count++;
          yaw_debug.can3.feedback_last_gap_interval_ms = interval;
          yaw_debug.can3.feedback_last_gap_tick_ms = now;
        }
      }
      else
        can3_yaw_feedback_seen = 1U;
      can3_yaw_feedback_last_rx_tick = now;

      FDCAN_ErrorCountersTypeDef error_counters;
      HAL_FDCAN_GetErrorCounters(hfdcan, &error_counters);
      yaw_debug.can3.tx_error_count = error_counters.TxErrorCnt;
      yaw_debug.can3.rx_error_count = error_counters.RxErrorCnt;
      /* CEL 在读取 ECR 后清零，因此在软件中累加，得到上电以来的协议错误总数。 */
      yaw_debug.can3.error_logging_count += error_counters.ErrorLogging;
    }

    if (BoardLink_RxDispatch(hfdcan, (uint16_t)rx_header.Identifier, rx_data))
      continue;  // 双板消息处理完后继续取 FIFO 中的下一帧
    // 超电帧
    if ((rx_header.Identifier == Supercap_receive_id) ||
        (rx_header.Identifier == Supercap_chassis_power_id))
      Supercup_DecodeCandata(hfdcan, rx_data,rx_header.Identifier);
    //IMU帧
     if(hfdcan == dm_imu_gimbal.can_handle && rx_header.Identifier == dm_imu_gimbal.mst_id)
      IMU_UpdateData(rx_data, &dm_imu_gimbal);
    // 电机帧
    DJIMotor_DecodeCandata(hfdcan, rx_header.Identifier, rx_data);
    DMMotor_DecodeCandata(hfdcan, rx_header.Identifier, rx_data);
    /* DMMotor124_DecodeCandata(hfdcan, rx_header.Identifier, rx_data); */
  }
}

/**
 * @brief CAN错误处理回调函数，重启相关设备
 *
 * @param hfdcan
 */
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
  FDCAN_FilterTypeDef fdcan_filter;

  if (hfdcan == &hfdcan3)
    yaw_debug.can3.error_recovery_count++;

  fdcan_filter.IdType = FDCAN_STANDARD_ID;             // 过滤标准ID
  fdcan_filter.FilterIndex = 0;                        // 滤波器索引
  fdcan_filter.FilterType = FDCAN_FILTER_MASK;         // 掩码模式
  fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 过滤器0关联到FIFO0
  fdcan_filter.FilterID1 = 0x00000000;                 // 不去过滤任何ID
  fdcan_filter.FilterID2 = 0x00000000;                 // 同上

  HAL_FDCAN_Stop(hfdcan);
  HAL_FDCAN_DeInit(hfdcan);
  HAL_FDCAN_Init(hfdcan);
  HAL_FDCAN_ConfigFilter(hfdcan, &fdcan_filter); // 将上述配置到CAN
  HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  uint32_t notifications = FDCAN_IT_RX_FIFO0_NEW_MESSAGE;
  if (hfdcan == &hfdcan2 || hfdcan == &hfdcan3)
    notifications |= FDCAN_IT_RX_FIFO0_FULL | FDCAN_IT_RX_FIFO0_MESSAGE_LOST;
  if (hfdcan == &hfdcan3)
    notifications |= CAN3_ERROR_STATUS_NOTIFICATIONS;
  HAL_FDCAN_ActivateNotification(hfdcan, notifications, 0);
  HAL_FDCAN_Start(hfdcan);
}

/**
 * @brief 记录 FDCAN3 进入 warning/passive/bus-off 的状态，不在中断内重启总线
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
  if (hfdcan != &hfdcan3)
    return;

  FDCAN_ProtocolStatusTypeDef protocol_status;
  FDCAN_ErrorCountersTypeDef error_counters;

  yaw_debug.can3.error_status_count++;
  yaw_debug.can3.last_error_status_its = ErrorStatusITs;
  yaw_debug.can3.error_status_last_tick_ms = HAL_GetTick();

  HAL_FDCAN_GetProtocolStatus(hfdcan, &protocol_status);
  HAL_FDCAN_GetErrorCounters(hfdcan, &error_counters);
  yaw_debug.can3.last_error_code = protocol_status.LastErrorCode;
  yaw_debug.can3.tx_error_count = error_counters.TxErrorCnt;
  yaw_debug.can3.rx_error_count = error_counters.RxErrorCnt;
  yaw_debug.can3.error_logging_count += error_counters.ErrorLogging;
}

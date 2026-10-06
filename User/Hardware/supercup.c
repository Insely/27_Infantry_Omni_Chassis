/**
 * @file supercup.c
 * @author Siri (lixirui@outlook.com)
 * @brief 超电相关控制
 * @version 0.1
 * @date 2024-10-25
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "supercup.h"
#include "string.h"
#include "stm32h7xx_hal.h"

Supercap_t cap;
Supercap_send_t send_data_cap;
FDCAN_HandleTypeDef *Supercup_send_hfdcan; // 通过此处自动确定超电挂载的can通道

/**
 * @brief 解码超电信息
 *
 * @param hfdcan 收到超电信息的can通道
 * @param data 收到的数据
 */
void Supercup_DecodeCandata(FDCAN_HandleTypeDef *hfdcan, uint8_t *data, uint32_t receive_id)
{
    // 确定超电所挂载的can通道
    Supercup_send_hfdcan = hfdcan;

    // 获取超电电压
    static float rev_vol;
    static float chassis_power;
    if (receive_id == Supercap_receive_id)
    {
        memcpy(&rev_vol, data, sizeof(rev_vol));
        cap.remain_vol = rev_vol;
    }
    if (receive_id == Supercap_chassis_power_id)
    {
        memcpy(&chassis_power, data, sizeof(chassis_power));
        cap.Chassis_power = chassis_power;
    }
}

/**
 * @brief 发送超电控制数据
 *
 */
void Supercup_SendData(void)
{
    static uint8_t can_send_data[8];
    static FDCAN_TxHeaderTypeDef tx_message;

    tx_message.Identifier = Supercap_send_id;
    tx_message.IdType = FDCAN_STANDARD_ID;              // 标准ID
    tx_message.TxFrameType = FDCAN_DATA_FRAME;          // 数据帧
    tx_message.DataLength = FDCAN_DATA_BYTES_8;         // 发送数据长度
    tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;  // 设置错误状态指示
    tx_message.BitRateSwitch = FDCAN_BRS_OFF;           // 不开启可变波特率
    tx_message.FDFormat = FDCAN_CLASSIC_CAN;            // 普通CAN格式
    tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 用于发送事件FIFO控制, 不存储
    tx_message.MessageMarker = 0x00;                    // 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0到0xFF

    Supercap_send_t send_data;

    send_data.setPower = cap.set_max_power;
    send_data.cacheEnergy = cap.cache_energy;
    send_data.cacheEnergylimit = cap.cacheEnergylimit;

    memcpy(can_send_data, &send_data, sizeof(Supercap_send_t));

    HAL_FDCAN_AddMessageToTxFifoQ(Supercup_send_hfdcan, &tx_message, can_send_data);
}

/**
 * @brief 设置超电的功率
 *
 * @param set 设置值 50~200
 * @return int 是否设置成功 0成功，-1失败
 */
int Supercap_SetPower(uint8_t set)
{
    // if (set > 200 || set < 50)
    //	return -1;
    cap.set_max_power = set;
    return 0;
}

/**
 * @brief 获取超电剩余电压
 *
 * @return float 超电剩余电压
 */
float Supercap_GetRemainVol(void)
{
    return cap.remain_vol;
}

/**
 * @brief 获取超电预测剩余容量
 *
 * @return float 超电剩余容量 百分值
 */
float Supercap_GetPredictEnergy(void)
{
    return cap.prediect_energy;
}
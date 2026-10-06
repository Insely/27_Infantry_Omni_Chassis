/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2026-01-09 04:13:30
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-01-09 04:13:50
 * @FilePath: \Regular_Sentry_Gimbal\User\Hardware\IMU\hi14.c
 */

#include "hi14.h"
#include "string.h"
#include "stm32h7xx_hal.h"

FDCAN_HandleTypeDef *HI14_send_hfdcan; // 通过此处自动确定HI14挂载的can通道

HI14_data_t hi14;

/**
 * @brief 解码HI14信息
 *
 * @param hfdcan 收到HI14信息的can通道
 * @param data 收到的数据
 * @param receive_id CANid
 * @param IMU_ID 设备id
 */
void HI14_decode_candata(FDCAN_HandleTypeDef *hfdcan, uint8_t *data, uint32_t receive_id, uint8_t IMU_ID)
{
    // 确定超电所挂载的can通道

    if (receive_id - IMU_ID == 0x180) // 加速度帧
    {
        HI14_send_hfdcan = hfdcan;
        hi14.acceleration.HI14_Ax = 0.001f * (int16_t)(data[0] | (data[1] << 8));
        hi14.acceleration.HI14_Ay = 0.001f * (int16_t)(data[2] | (data[3] << 8));
        hi14.acceleration.HI14_Az = 0.001f * (int16_t)(data[4] | (data[5] << 8));
    }
    if (receive_id - IMU_ID == 0x280) // 角速度帧
    {
        HI14_send_hfdcan = hfdcan;
        hi14.angular_velocity.HI14_Dx = 0.1f * (int16_t)(data[0] | (data[1] << 8));
        hi14.angular_velocity.HI14_Dy = 0.1f * (int16_t)(data[2] | (data[3] << 8));
        hi14.angular_velocity.HI14_Dz = 0.1f * (int16_t)(data[4] | (data[5] << 8));
    }
    if (receive_id - IMU_ID == 0x380) // 欧拉角帧
    {
        HI14_send_hfdcan = hfdcan;
        hi14.Cape_Euler.HI14_roll = 0.01f * (int16_t)(data[0] | (data[1] << 8));
        hi14.Cape_Euler.HI14_pitch = 0.01f * (int16_t)(data[2] | (data[3] << 8));
        hi14.Cape_Euler.HI14_yaw = 0.01f * (int16_t)(data[4] | (data[5] << 8));
    }
    if (receive_id - IMU_ID == 0x480) // 四元数帧
    {
        HI14_send_hfdcan = hfdcan;
        hi14.quaternion.HI14_qw = 0.0001f * (int16_t)(data[0] | (data[1] << 8));
        hi14.quaternion.HI14_qx = 0.0001f * (int16_t)(data[2] | (data[3] << 8));
        hi14.quaternion.HI14_qy = 0.0001f * (int16_t)(data[4] | (data[5] << 8));
        hi14.quaternion.HI14_qz = 0.0001f * (int16_t)(data[6] | (data[7] << 8));
    }
    // if(receive_id - IMU_ID == 0x580)
    // {

    // }
    if (receive_id - IMU_ID == 0x680) // 气压帧
    {
        HI14_send_hfdcan = hfdcan;
        hi14.atmospheric_pressure = (int32_t)((uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24));
    }
    if (receive_id - IMU_ID == 0x780) // 倾角仪输出帧
    {
        HI14_send_hfdcan = hfdcan;
        hi14.inclination.HI14_xAngle = 0.01f * (int32_t)((uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24));
        hi14.inclination.HI14_yAngle = 0.01f * (int32_t)((uint32_t)data[4] | ((uint32_t)data[5] << 8) | ((uint32_t)data[6] << 16) | ((uint32_t)data[7] << 24));
    }
}
/**
 * @brief 修改HI14的ID
 * @param ID 修改前的id
 * @param target_ID 修改后的id
 */
void HI14_ID_Change(uint8_t ID, uint8_t target_ID)
{
    static uint8_t can_send_data[8];
    static FDCAN_TxHeaderTypeDef tx_message;
    // ID = HI14_ID;
    tx_message.Identifier = HI14_send_id + ID;

    tx_message.IdType = FDCAN_STANDARD_ID;              // 标准ID
    tx_message.TxFrameType = FDCAN_DATA_FRAME;          // 数据帧
    tx_message.DataLength = FDCAN_DLC_BYTES_8;          // 发送数据长度
    tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;  // 设置错误状态指示
    tx_message.BitRateSwitch = FDCAN_BRS_OFF;           // 不开启可变波特率
    tx_message.FDFormat = FDCAN_CLASSIC_CAN;            // 普通CAN格式
    tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 用于发送事件FIFO控制, 不存储
    tx_message.MessageMarker = 0x00;                    // 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0到0xFF

    can_send_data[0] = 0x23;
    can_send_data[1] = 0xA0;
    can_send_data[2] = 0x20;
    can_send_data[3] = 0x00;
    can_send_data[4] = target_ID; // ID
    can_send_data[5] = 0x00;
    can_send_data[6] = 0x00;
    can_send_data[7] = 0x00;
    HAL_FDCAN_AddMessageToTxFifoQ(HI14_send_hfdcan, &tx_message, can_send_data); // 修改id

    can_send_data[0] = 0x23;
    can_send_data[1] = 0x00;
    can_send_data[2] = 0x20;
    can_send_data[3] = 0x00;
    can_send_data[4] = 0x00;
    can_send_data[5] = 0x00;
    can_send_data[6] = 0x00;
    can_send_data[7] = 0x00;
    HAL_FDCAN_AddMessageToTxFifoQ(HI14_send_hfdcan, &tx_message, can_send_data); // flash保存

    can_send_data[0] = 0x23;
    can_send_data[1] = 0x00;
    can_send_data[2] = 0x20;
    can_send_data[3] = 0x00;
    can_send_data[4] = 0xFF;
    can_send_data[5] = 0x00;
    can_send_data[6] = 0x00;
    can_send_data[7] = 0x00;
    HAL_FDCAN_AddMessageToTxFifoQ(HI14_send_hfdcan, &tx_message, can_send_data); // 复位
}
/**
 * @brief 修改HI14各种参数的发送频率
 * @param ID 设备的ID
 * @param parameter 要修改的参数
 * @param frequency 频率
 */
void HI14_frequency_change(uint8_t ID, uint8_t parameter, uint8_t frequency)
{
    static uint8_t can_send_data[8];
    static FDCAN_TxHeaderTypeDef tx_message;
    // ID = HI14_ID;
    tx_message.Identifier = HI14_send_id + ID;

    tx_message.IdType = FDCAN_STANDARD_ID;              // 标准ID
    tx_message.TxFrameType = FDCAN_DATA_FRAME;          // 数据帧
    tx_message.DataLength = FDCAN_DLC_BYTES_8;          // 发送数据长度
    tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;  // 设置错误状态指示
    tx_message.BitRateSwitch = FDCAN_BRS_OFF;           // 不开启可变波特率
    tx_message.FDFormat = FDCAN_CLASSIC_CAN;            // 普通CAN格式
    tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 用于发送事件FIFO控制, 不存储
    tx_message.MessageMarker = 0x00;                    // 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0到0xFF

    can_send_data[0] = 0x2B;
    switch (parameter)
    {
    case Cceleration:
        can_send_data[1] = 0x00;
        break;
    case Angular_velocity:
        can_send_data[1] = 0x01;
        break;
    case Cape_Euler:
        can_send_data[1] = 0x02;
        break;
    case Quaternion:
        can_send_data[1] = 0x03;
        break;
    case Atmospheric_pressure:
        can_send_data[1] = 0x04;
        break;
    default:
        break;
    }

    can_send_data[2] = 0x18;
    can_send_data[3] = 0x05;
    switch (frequency)
    {
    case HI14_0_HZ:
        can_send_data[4] = 0x00;
        break;
    case HI14_10_HZ:
        can_send_data[4] = 0x64;
        break;
    case HI14_20_HZ:
        can_send_data[4] = 0x32;
        break;
    case HI14_50_HZ:
        can_send_data[4] = 0x14;
        break;
    case HI14_100_HZ:
        can_send_data[4] = 0x0A;
        break;
    case HI14_200_HZ:
        can_send_data[4] = 0x05;
        break;
    default:
        break;
    }
    can_send_data[5] = 0x00;
    can_send_data[6] = 0x00;
    can_send_data[7] = 0x00;
    HAL_FDCAN_AddMessageToTxFifoQ(HI14_send_hfdcan, &tx_message, can_send_data); // 修改id

    can_send_data[0] = 0x23;
    can_send_data[1] = 0x00;
    can_send_data[2] = 0x20;
    can_send_data[3] = 0x00;
    can_send_data[4] = 0x00;
    can_send_data[5] = 0x00;
    can_send_data[6] = 0x00;
    can_send_data[7] = 0x00;
    HAL_FDCAN_AddMessageToTxFifoQ(HI14_send_hfdcan, &tx_message, can_send_data); // flash保存

    can_send_data[0] = 0x23;
    can_send_data[1] = 0x00;
    can_send_data[2] = 0x20;
    can_send_data[3] = 0x00;
    can_send_data[4] = 0xFF;
    can_send_data[5] = 0x00;
    can_send_data[6] = 0x00;
    can_send_data[7] = 0x00;
    HAL_FDCAN_AddMessageToTxFifoQ(HI14_send_hfdcan, &tx_message, can_send_data); // 复位
}
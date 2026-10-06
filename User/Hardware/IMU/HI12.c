/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2026-01-27 16:02:49
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-01-28 09:31:12
 * @FilePath: \Regular_Sentry_Gimbal\User\Hardware\IMU\HI12.c
 */

 #include "HI12.h"
 #include <string.h>

 can_sensor_data_t HI12_Data;
static FDCAN_HandleTypeDef *HI12_CanHandle = NULL;

// 调试计数器：添加到 Watch 窗口观察
volatile uint32_t DBG_HI12_Total = 0;   // 总接收次数
volatile uint32_t DBG_HI12_Accel = 0;   // 加速度包接收次数
volatile uint32_t DBG_Last_PGN = 0;     // 记录上一次收到的PGN

void HI12_Init(FDCAN_HandleTypeDef *hcan)
{
    HI12_CanHandle = hcan;
}
static int Parse_Time(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    const uint8_t *raw = frame->data;
    data->utc_year = raw[0];
    data->utc_month = raw[1];
    data->utc_day = raw[2];
    data->hours = raw[3];
    data->minutes = raw[4];
    data->seconds = raw[5];
    data->milliseconds = raw[6] | (raw[7] << 8);
    data->timestamp_ms = (uint32_t)data->hours * 3600000UL + (uint32_t)data->minutes * 60000UL + (uint32_t)data->seconds * 1000UL + data->milliseconds;
    return CAN_MSG_TIME;
}

static int Parse_Accel(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    int16_t *raw = (int16_t*)frame->data;
    data->acc_x = raw[0] * 0.00048828f;
    data->acc_y = raw[1] * 0.00048828f;
    data->acc_z = raw[2] * 0.00048828f;
    return CAN_MSG_ACCEL;
}

static int Parse_Gyro(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    int16_t *raw = (int16_t*)frame->data;
    data->gyr_x = raw[0] * 0.061035f;
    data->gyr_y = raw[1] * 0.061035f;
    data->gyr_z = raw[2] * 0.061035f;
    return CAN_MSG_GYRO;
}

static int Parse_Pitch_Roll(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    int32_t *raw = (int32_t*)frame->data;
    data->roll = raw[0] * 0.001f;
    data->pitch = raw[1] * 0.001f;
    return CAN_MSG_PITCH_ROLL;
}


static int Parse_Yaw(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    int32_t *raw = (int32_t*)frame->data;
    data->imu_yaw = raw[0] * 0.001f;
    return CAN_MSG_YAW;
}

static int Parse_MAG(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    int16_t *raw = (int16_t*)frame->data;
    data->mag_x = raw[0] * 0.030517f;
    data->mag_y = raw[1] * 0.030517f;
    data->mag_z = raw[2] * 0.030517f;
    return CAN_MSG_MAG;
}


static int Parse_Quat(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    int16_t *raw = (int16_t*)frame->data;
    data->quat_w = raw[0] * 0.0001f;
    data->quat_x = raw[1] * 0.0001f;
    data->quat_y = raw[2] * 0.0001f;
    data->quat_z = raw[3] * 0.0001f;
    return CAN_MSG_QUAT;
}

static int Parse_Inclination(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    int32_t *raw = (int32_t*)frame->data;
    data->incli_x = raw[0] * 0.001f;
    data->incli_y = raw[1] * 0.001f;
    return CAN_MSG_INCLI;
}

int Parse_Frame(uint32_t raw_id,uint8_t *data_ptr, can_sensor_data_t *data)
{
/*     if (!frame || !data) return CAN_MSG_ERROR;
    if (!(frame->can_id & HIPNUC_CAN_EFF_FLAG)) return CAN_MSG_UNKNOWN;
    uint32_t id = frame->can_id & HIPNUC_CAN_EFF_MASK; */
    uint32_t pgn = (raw_id >> 8) & 0xFFFF;
    DBG_Last_PGN = pgn; // 调试用：看看到底收到了什么PGN
    hipnuc_can_frame_t temp_frame;
    memcpy(temp_frame.data, data_ptr, 8);
    switch (pgn) {
        case PGN_UTC_TIME:         return Parse_Time(&temp_frame, data);
        case PGN_ACCEL:            
            DBG_HI12_Accel++; // 调试：收到加速度包+1
            return Parse_Accel(&temp_frame, data);
        case PGN_GYRO:             return Parse_Gyro(&temp_frame, data);
        case PGN_EULER_RP:         return Parse_Pitch_Roll(&temp_frame, data);
        case PGN_EULER_YAW:        return Parse_Yaw(&temp_frame, data);
        case PGN_MAGNETOMETER:     return Parse_MAG(&temp_frame, data);
        case PGN_QUATERNION:       return Parse_Quat(&temp_frame, data);
        case PGN_INCLINOMETER:     return Parse_Inclination(&temp_frame, data);
        default:                   return CAN_MSG_UNKNOWN;
    }
}

void HI12_Decode(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data)
{
    // 1. 安全检查
    if (HI12_CanHandle == NULL) return; // 如果 main.c 初始化顺序错了，这里会挡住
    if (hfdcan != HI12_CanHandle) return;

    // 2. 必须是扩展帧 (J1939 标准 )
    if (rx_header->IdType != FDCAN_EXTENDED_ID) return;

    DBG_HI12_Total++; // 调试：总计数+1

    // 3. 直接解析
    Parse_Frame(rx_header->Identifier, rx_data, &HI12_Data);
}

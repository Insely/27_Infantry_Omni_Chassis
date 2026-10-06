/*
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2025-11-03 00:07:24
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-03-19 11:50:10
 * @FilePath: \Season26_Regular_Sentry_Gimbal\User\Hardware\Motor\motor.c
 */
/**
 * @file motor.c
 * @author Wang Zihao
 * @brief 各类电机控制与反馈
 * @version 0.1
 * @date 2025-9-12
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "motor.h"
#include "math.h"
#include "string.h"

#define abs(a) a > 0 ? a : -a

#if (USE_DJIMotor == 1)
// 电机数据定义
DJI_motor_data_s DJIMotor_data[QUANTITY_OF_CAN][QUANTITY_OF_DJIMOTOR];

/**
 * @brief 大疆电机初始化，暂时只用来初始化类型
 *
 * @param motor_type 电机种类
 * @param motor_id 电机can通道及ID
 */
void DJIMotor_Init(Motor_Type_e motor_type, DJIcan_id motor_id)
{
    uint8_t cantype = motor_id / 11; // 获得电机所在can路
    uint8_t canid = motor_id % 11;   // 得到电机ID值；

    DJIMotor_data[cantype][canid].Motor_type = motor_type; // 初始化相应电机
}

/**
 * @brief 设置大疆电机零点
 *
 * @param zero_angle 零点角度
 * @param motor_id 电机ID
 */
void DJIMotor_SetZero(double zero_angle, DJIcan_id motor_id)
{
    uint8_t cantype = motor_id / 11; // 获得电机所在can路
    uint8_t canid = motor_id % 11;   // 得到电机ID值；

    DJIMotor_data[cantype][canid].angle_zero = zero_angle; // 初始化相应电机
}

/**
 * @brief 设置大疆电机电流
 *
 * @param val 电流值
 * @param motor_id 电机can通道及ID
 */
void DJIMotor_Set(int16_t val, DJIcan_id motor_id)
{
    DJIMotor_data[motor_id / 11][motor_id % 11].set = val; // 设置电流
}

/**
 * @brief 获取大疆电机数据
 *
 * @param motor_id 电机can通道及ID
 * @return DJI_motor_data_s 电机数据结构体。
 */
DJI_motor_data_s DJIMotor_GetData(DJIcan_id motor_id) // 获取马达数据
{
    return DJIMotor_data[motor_id / 11][motor_id % 11];
}

/**
 * @brief 大疆电机CAN数据接受以及处理
 *
 * @param ptr 电机数据
 * @param data can数据
 */
void DJIMotor_GetProcessMotorData(DJI_motor_data_s *ptr, uint8_t data[])
{
    // get raw data
    (ptr)->last_ecd = (ptr)->ecd;
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);
    (ptr)->given_current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    (ptr)->temperate = (data)[6];

    // process the data
    // count cnt
    if ((ptr)->last_ecd > 7000 && (ptr)->ecd < 1000)
        (ptr)->ecd_cnt += ((ECD_MAX - (ptr)->last_ecd) + (ptr)->ecd);
    else if ((ptr)->last_ecd < 1000 && (ptr)->ecd > 7000)
        (ptr)->ecd_cnt -= ((ECD_MAX - (ptr)->ecd) + (ptr)->last_ecd);
    else
        (ptr)->ecd_cnt += ((ptr)->ecd - (ptr)->last_ecd);
    // process data
    (ptr)->angle_cnt = (ptr)->ecd_cnt * ECD_TO_ANGEL_DJI;
    // 根据不同电机进行不同处理
    if ((ptr)->Motor_type == DJI_GM6020)
    {
        // 计算出轴转速
        (ptr)->round_speed = (ptr)->speed_rpm;

        // 计算相对角度 -180~180 谨防精度丢失 总角度过大时
        float angle = (ptr)->angle_cnt - (ptr)->angle_zero;
        uint32_t mul = abs((int)angle) / 180;
        if (angle > 180.0f)
        {
            if (mul % 2 == 1) // 处于-180度
                angle -= (mul + 1) * 180;
            else // 处于180度
                angle -= mul * 180;
        }
        if (angle < -180.0f)
        {
            if (mul % 2 == 1) // 处于180度
                angle += (mul + 1) * 180;
            else // 处于-180度
                angle += mul * 180;
        }
        (ptr)->angle = angle;
    }
    else if ((ptr)->Motor_type == DJI_M3508)
    {
        (ptr)->round_speed = (ptr)->speed_rpm / M3508_P;
    }
    else if ((ptr)->Motor_type == DJI_M2006)
    {
        (ptr)->round_speed = (ptr)->speed_rpm / M2006_P;
    }
}

/**
 * @brief 大疆电机can数据处理
 *
 * @param hfdcan CAN通道
 * @param id can标识符
 * @param data can数据
 */
void DJIMotor_DecodeCandata(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data)
{
    if (id < CAN_ID1)
        return;
    if (id - CAN_ID1 < QUANTITY_OF_DJIMOTOR) // 防止数组溢出
    {
        if (hfdcan == &hfdcan1)
        {
            DJIMotor_GetProcessMotorData(&DJIMotor_data[0][id - CAN_ID1], data);
        }
        else if (hfdcan == &hfdcan2)
        {
            DJIMotor_GetProcessMotorData(&DJIMotor_data[1][id - CAN_ID1], data);
        }
        else if (hfdcan == &hfdcan3)
        {
            DJIMotor_GetProcessMotorData(&DJIMotor_data[2][id - CAN_ID1], data);
        }
    }
}

/**
 * @brief 大疆电机电流值发送，建议放freertos里定期发送
 *
 */
void DJIMotor_SendCurrent(DJIcan_send_id_e CAN_Send_ID, DJIcan_send_id_e CAN_Type)
{
    uint8_t can_send_data[8];
    uint8_t canid;
    uint16_t identifier;
    FDCAN_HandleTypeDef *hcan;

    /*     if (hfdcan1.ErrorCode)
            HAL_FDCAN_ErrorCallback(&hfdcan1);
        if (hfdcan2.ErrorCode)
            HAL_FDCAN_ErrorCallback(&hfdcan2);
        if (hfdcan3.ErrorCode)
            HAL_FDCAN_ErrorCallback(&hfdcan3); */

    // 判断电机类型以及电机ID
    switch (CAN_Send_ID)
    {
    case CAN_20063508_1_4_ID:
    {
        identifier = CAN_20063508_1_4_send_ID;
        canid = 0;
        break;
    }
    case CAN_20063508_5_8_ID:
    {
        identifier = CAN_20063508_5_8_send_ID;
        canid = 4;
        break;
    }
    case CAN_6020_1_4_ID:
    {
        identifier = CAN_6020_1_4_send_ID;
        canid = 4;
        break;
    }
    case CAN_6020_5_7_ID:
    {
        identifier = CAN_6020_5_7_send_ID;
        canid = 8;
        break;
    }
    default:
        return;
    }

    // 判断CAN路
    switch (CAN_Type)
    {
    case DJI_CAN_1:
        hcan = &hfdcan1;
        break;
    case DJI_CAN_2:
        hcan = &hfdcan2;
        break;
    case DJI_CAN_3:
        hcan = &hfdcan3;
        break;
    default:
        return;
    }
    if (hcan->ErrorCode)
        HAL_FDCAN_ErrorCallback(hcan);
    // 三路can依次赋值
    for (int i = 0; i < 8; i += 2)
    {
        can_send_data[i] = (DJIMotor_data[CAN_Type][canid].set >> 8);
        can_send_data[i + 1] = DJIMotor_data[CAN_Type][canid].set;
        canid++;
    }

    // 发送CAN数据
    Fdcanx_SendData(hcan, identifier, can_send_data, 8);
}

#endif // USE_DJIMotor

#if (USE_DMMotor == 1)

// 达妙电机数据定义
DM_motor_data_s DM_Motor_data[QUANTITY_OF_CAN][QUANTITY_OF_DMMOTOR];

static uint8_t yaw_motor_was_non1 = 0U;

/**
 * @brief 使能达妙电机
 * @param motor_id 电机ID
 */
void DMMotor_Enable(DMcan_id motor_id)
{
    if (motor_id >= DM_MOTOR_NUM)
        return;

    uint8_t cantype = motor_id / 6; // 获得电机所在can路
    uint8_t canid = motor_id % 6;   // 得到电机ID值；

    switch (cantype)
    {
    case 0:
        dm_motor_enable(&hfdcan1, &DM_Motor_data[cantype][canid].motor_data);
        break;
    case 1:
        dm_motor_enable(&hfdcan2, &DM_Motor_data[cantype][canid].motor_data);
        break;
    case 2:
        dm_motor_enable(&hfdcan3, &DM_Motor_data[cantype][canid].motor_data);
        break;
    default:
        break;
    }
}

/**
 * @brief 失能达妙电机
 * @param motor_id 电机ID
 */
void DMMotor_Disable(uint8_t motor_id)
{
    if (motor_id >= DM_MOTOR_NUM)
        return;
    uint8_t cantype = motor_id / 6; // 获得电机所在can路
    uint8_t canid = motor_id % 6;   // 得到电机ID值；
    dm_motor_disable(Get_CanHandle(cantype), &DM_Motor_data[cantype][canid].motor_data);
}

/**
 * @brief 设置达妙电机控制模式
 * @param motor_id 电机ID
 * @param mode 控制模式
 */
void DMMotor_SetMode(DMcan_id motor_id, DM_mode_e mode)
{
    if (motor_id >= DM_MOTOR_NUM)
        return;

    uint8_t cantype = motor_id / 6; // 获得电机所在can路
    uint8_t canid = motor_id % 6;   // 得到电机ID值；

    DM_Motor_data[cantype][canid].motor_data.ctrl.mode = mode;
}

/**
 * @brief 达妙电机初始化
 * @param motor_id 电机ID (0-5)
 */
void DMMotor_Init(Motor_Type_e motor_type, DMcan_id motor_id)
{
    if (motor_id >= DM_MOTOR_NUM)
        return;

    uint8_t cantype = motor_id / 6; // 获得电机所在can路
    uint8_t canid = motor_id % 6;   // 得到电机ID值；

    memset(&DM_Motor_data[cantype][canid], 0, sizeof(DM_motor_data_s));

    DM_Motor_data[cantype][canid].motor_data.id = canid + 1;        // CAN_ID：0x01～0x06
    DM_Motor_data[cantype][canid].motor_data.mst_id = canid + DM_FEEDBACK_ID_BASE; // MASTER_ID：0x11～0x16
    DM_Motor_data[cantype][canid].motor_data.tmp.read_flag = 1;
    DM_Motor_data[cantype][canid].motor_data.tmp.PMAX = P_MAX;
    DM_Motor_data[cantype][canid].motor_data.tmp.VMAX = V_MAX;
    DM_Motor_data[cantype][canid].motor_data.tmp.TMAX = T_MAX;
    DM_Motor_data[cantype][canid].motor_data.ctrl.mode = mit_mode; // 默认MIT模式

    // 使能电机
    DMMotor_Enable(motor_id);
}

/**
 * @brief 设置达妙电机控制参数
 * @param motor_id 电机ID
 * @param pos 位置设定值 (rad)
 * @param vel 速度设定值 (rad/s)
 * @param tor 扭矩设定值 (N*M)
 * @param kp 位置比例增益(N/r)
 * @param kd 位置微分增益(N*s/r)
 */
void DMMotor_Set(DMcan_id motor_id, float pos, float vel, float tor, float kp, float kd)
{
    if (motor_id >= DM_MOTOR_NUM)
        return;

    uint8_t cantype = motor_id / 6; // 获得电机所在can路
    uint8_t canid = motor_id % 6;   // 得到电机ID值；

    DM_motor_t *m = &DM_Motor_data[cantype][canid].motor_data;
    m->ctrl.pos_set = pos;
    m->ctrl.vel_set = vel;
    m->ctrl.tor_set = tor;
    m->ctrl.kp_set = kp;
    m->ctrl.kd_set = kd;
}

/**
 * @brief 设置达妙电机零点
 * @param zero_angle 零点角度
 * @param motor_id 电机ID
 */
void DMMotor_SetZero(double zero_angle, DMcan_id motor_id)
{
    uint8_t cantype = motor_id / 6; // 获得电机所在can路
    uint8_t canid = motor_id % 6;   // 得到电机ID值；

    DM_Motor_data[cantype][canid].motor_data.angle_zero = zero_angle; // 初始化相应电机
}

/**
 * @brief 达妙电机CAN数据处理
 * @param hfdcan CAN通道
 * @param id can标识符
 * @param data can数据
 */
void DMMotor_DecodeCandata(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data)
{
    if (id < DM_FEEDBACK_ID_BASE || id >= DM_FEEDBACK_ID_BASE + QUANTITY_OF_DMMOTOR)
        return;
    uint8_t cantype;
    uint8_t canid;
    id -= DM_FEEDBACK_ID_BASE;
    if (hfdcan == &hfdcan1)
    { // 达妙电机反馈ID
        cantype = 0;
    }
    else if (hfdcan == &hfdcan2)
    {
        cantype = 1;
    }
    else if (hfdcan == &hfdcan3)
    {
        cantype = 2;
    }
    else
    {
        return;
    }
    canid = id;
    // 更新所有达妙电机的反馈数据
    dm_motor_fbdata(&DM_Motor_data[cantype][canid].motor_data, data);
    DM_Motor_data[cantype][canid].feedback_tick=HAL_GetTick();
    DM_Motor_data[cantype][canid].feedback_ready=1;

    if (cantype == (uint8_t)(YAWMotor / 6) && canid == (uint8_t)(YAWMotor % 6))
    {
        DM_motor_t *yaw = &DM_Motor_data[cantype][canid].motor_data;
        float abs_torque = fabsf(yaw->para.tor);
        float abs_velocity_error = fabsf(yaw->ctrl.vel_set - yaw->para.vel);

        if (yaw->para.Tmos > yaw_debug.motor.max_tmos)
            yaw_debug.motor.max_tmos = yaw->para.Tmos;
        if (yaw->para.Tcoil > yaw_debug.motor.max_tcoil)
            yaw_debug.motor.max_tcoil = yaw->para.Tcoil;
        if (abs_torque > yaw_debug.motor.max_abs_torque)
            yaw_debug.motor.max_abs_torque = abs_torque;
        if (abs_velocity_error > yaw_debug.motor.max_abs_velocity_error)
        {
            yaw_debug.motor.max_abs_velocity_error = abs_velocity_error;
            yaw_debug.motor.max_abs_velocity_error_tick_ms = DM_Motor_data[cantype][canid].feedback_tick;
        }

        if (yaw->para.state != 1)
        {
            yaw_debug.motor.non1_feedback_count++;
            if (!yaw_motor_was_non1)
                yaw_debug.motor.non1_event_count++;
            yaw_motor_was_non1 = 1U;
            yaw_debug.motor.last_non1_tick_ms = DM_Motor_data[cantype][canid].feedback_tick;
            yaw_debug.motor.last_non1_state = yaw->para.state;
            yaw_debug.motor.last_non1_tmos = yaw->para.Tmos;
            yaw_debug.motor.last_non1_tcoil = yaw->para.Tcoil;
            yaw_debug.motor.last_non1_torque = yaw->para.tor;
            yaw_debug.motor.last_non1_velocity = yaw->para.vel;
            yaw_debug.motor.last_non1_velocity_set = yaw->ctrl.vel_set;
        }
        else
        {
            yaw_motor_was_non1 = 0U;
        }
    }
}

/**
 * @brief 达妙电机控制命令发送
 */
int DMMotor_SendCtrl(DMcan_id motor_id)
{
    static uint8_t yaw_tx_call_seen = 0U;
    static uint32_t yaw_tx_call_last_tick = 0U;

    if (motor_id >= DM_MOTOR_NUM)
        return 0;

    if (motor_id == YAWMotor)
    {
        uint32_t now = HAL_GetTick();

        if (yaw_tx_call_seen != 0U)
        {
            yaw_debug.motor.tx_call_interval_ms = now - yaw_tx_call_last_tick;
            if (yaw_debug.motor.tx_call_interval_ms > yaw_debug.motor.tx_call_max_interval_ms)
                yaw_debug.motor.tx_call_max_interval_ms = yaw_debug.motor.tx_call_interval_ms;
            if (yaw_debug.motor.tx_call_interval_ms > 10U)
            {
                yaw_debug.motor.tx_call_gap_over_10ms_count++;
                yaw_debug.motor.tx_call_last_gap_tick_ms = now;
                yaw_debug.motor.tx_call_last_gap_interval_ms = yaw_debug.motor.tx_call_interval_ms;
            }
        }
        else
            yaw_tx_call_seen = 1U;
        yaw_tx_call_last_tick = now;
    }

    uint8_t cantype = motor_id / 6; // 获得电机所在can路
    uint8_t canid = motor_id % 6;   // 得到电机ID值；
    FDCAN_HandleTypeDef *hcan = Get_CanHandle(cantype);

    // if (DM_Motor_data[cantype][canid].motor_data.ctrl.mode != 0)
    if (DM_Motor_data[cantype][canid].motor_data.para.state == 1)
    { // 如果电机已使能
        dm_motor_ctrl_send(hcan, &DM_Motor_data[cantype][canid].motor_data);
        return 1;
    }
    else if (DM_Motor_data[cantype][canid].motor_data.para.state == 0)
    {
        DMMotor_Enable(motor_id);
        return 0;
    }
    else
    {
        dm_motor_clear_err(hcan, &DM_Motor_data[cantype][canid].motor_data);
        DMMotor_Disable(motor_id);
        return 0;
    }
}

/**
 * @brief 获取达妙电机数据
 *
 * @param motor_id 电机can通道及ID
 * @return DM_motor_data_s 电机数据结构体。
 */
DM_motor_data_s DMMotor_GetData(DMcan_id motor_id) // 获取马达数据
{
    return DM_Motor_data[motor_id / 6][motor_id % 6];
}

#endif // USE_DMMotor

#if (USE_DMMotor124 == 1)

// 电机数据定义
DM_motor124_data_s DMMotor124_data[QUANTITY_OF_CAN][QUANTITY_OF_DMMOTOR124];

/**
 * @brief达妙电机初始化
 *
 * @param motor_type 电机种类：J4310
 * @param motor_id 电机can通道及ID
 */
void DMMotor124_Init(DMMotor_Type_e motor_type, DM_can_id motor_id)
{
    uint8_t cantype = (motor_id + 1) / 8; // 获得电机所在can路
    uint8_t canid = motor_id % 8;         // 得到电机ID值；

    DMMotor124_data[cantype][canid].Motor_type = motor_type; // 初始化相应电机
}

void DMMotor124_SetZero(double zero_angle, DM_can_id motor_id)
{
    uint8_t cantype = motor_id / 8; // 获得电机所在can路
    uint8_t canid = motor_id % 8;   // 得到电机ID值；

    DMMotor124_data[cantype][canid].angle_zero = zero_angle; // 初始化相应电机
}

/**
 * @brief 设置DAMIAO电机电流
 *
 * @param val 电流值
 * @param motor_id 电机can通道及ID
 */
void DMMotor124_Set(int16_t val, DM_can_id motor_id)
{
    DMMotor124_data[(motor_id + 1) / 8][motor_id % 8].set = val; // 设置电流
}

/**
 * @brief 获取DAMIAO电机数据
 *
 * @param motor_id 电机can通道及ID
 * @return DJI_motor_data_s 电机数据结构体。
 */
DM_motor124_data_s DMMotor124_GetData(DM_can_id motor_id) // 获取马达数据
{
    return DMMotor124_data[(motor_id + 1) / 8][motor_id % 8];
}

/**
 * @brief DAMIAO电机CAN数据接受以及处理
 *
 * @param ptr 电机数据
 * @param data can数据
 */
void DMMotor124_GetProcessMotorData(DM_motor124_data_s *ptr, uint8_t data[])
{
    // get raw data
    (ptr)->last_ecd = (ptr)->ecd;
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);
    (ptr)->speed_rpm = 0.01f * (int16_t)((data)[2] << 8 | (data)[3]);
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);
    (ptr)->coil_temp = (data)[6];
    (ptr)->pcb_temp = (data)[7];

    // process the data
    // count cnt
    if ((ptr)->last_ecd > 7000 && (ptr)->ecd < 1000)
        (ptr)->ecd_cnt += ((ECD_MAX - (ptr)->last_ecd) + (ptr)->ecd);
    else if ((ptr)->last_ecd < 1000 && (ptr)->ecd > 7000)
        (ptr)->ecd_cnt -= ((ECD_MAX - (ptr)->ecd) + (ptr)->last_ecd);
    else
        (ptr)->ecd_cnt += ((ptr)->ecd - (ptr)->last_ecd);
    // process data
    (ptr)->angle_cnt = (ptr)->ecd_cnt * ECD_TO_ANGEL_DM;
    // 根据不同电机进行不同处理
    if ((ptr)->Motor_type == J4310)
    {
        // 计算出轴转速
        (ptr)->round_speed = (ptr)->speed_rpm;

        // 计算相对角度 -180~180 谨防精度丢失 总角度过大时
        float angle = (ptr)->angle_cnt - (ptr)->angle_zero;
        uint32_t mul = abs((int)angle) / 180;
        if (angle > 180.0f)
        {
            if (mul % 2 == 1) // 处于-180度
                angle -= (mul + 1) * 180;
            else // 处于180度
                angle -= mul * 180;
        }
        if (angle < -180.0f)
        {
            if (mul % 2 == 1) // 处于180度
                angle += (mul + 1) * 180;
            else // 处于-180度
                angle += mul * 180;
        }
        (ptr)->angle = angle;
    }
    else
    {
    }
}

/**
 * @brief DAMIAO电机can数据处理
 *
 * @param hfdcan CAN通道
 * @param id can标识符
 * @param data can数据
 */
void DMMotor124_DecodeCandata(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data)
{
    if (id - DM_ID1 <= 8 && id - DM_ID1 >= 0) // 防止数组溢出
    {
        if (hfdcan == &hfdcan1)
        {
            DMMotor124_GetProcessMotorData(&DMMotor124_data[0][id - DM_ID1], data);
        }
        else if (hfdcan == &hfdcan2)
        {
            DMMotor124_GetProcessMotorData(&DMMotor124_data[1][id - DM_ID1], data);
        }
        else if (hfdcan == &hfdcan3)
        {
            DMMotor124_GetProcessMotorData(&DMMotor124_data[2][id - DM_ID1], data);
        }
    }
}

/**
 * @brief DAMIAO电机电流值发送，建议放freertos里定期发送
 *
 */
void DMMotor124_SendCurrent(DMcan_send_id_e CAN_Send_ID, DMcan_send_id_e CAN_Type)
{
    uint8_t can_send_data[8];
    uint8_t canid;
    uint16_t identifier;
    FDCAN_HandleTypeDef *hcan;

    if (hfdcan1.ErrorCode)
        HAL_FDCAN_ErrorCallback(&hfdcan1);
    if (hfdcan2.ErrorCode)
        HAL_FDCAN_ErrorCallback(&hfdcan2);
    if (hfdcan3.ErrorCode)
        HAL_FDCAN_ErrorCallback(&hfdcan3);

    // 判断电机类型以及电机ID
    switch (CAN_Send_ID)
    {
    case DM_J4310_1_4_ID:
    {
        identifier = DM_J4310_1_4_send_ID;
        canid = 0;
        break;
    }

    case DM_J4310_5_8_ID:
    {
        identifier = DM_J4310_5_8_send_ID;
        canid = 4;
        break;
    }
    }

    // 判断CAN路
    switch (CAN_Type)
    {
    case DM_CAN_1:
        hcan = &hfdcan1;
        break;
    case DM_CAN_2:
        hcan = &hfdcan2;
        break;
    case DM_CAN_3:
        hcan = &hfdcan3;
        break;
    }

    // 三路can依次赋值
    for (int i = 0; i < 8; i += 2)
    {
        can_send_data[i] = DMMotor124_data[CAN_Type][canid].set;
        can_send_data[i + 1] = (DMMotor124_data[CAN_Type][canid].set >> 8);
        canid++;
    }
    // 发送CAN数据
    Fdcanx_SendData(hcan, identifier, can_send_data, 8);
}

#endif // USE_DMMotor

#if (USE_LZMotor == 1)
// 电机数据数组
LZ_Motor_t LZ_Motors[QUANTITY_OF_CAN][QUANTITY_OF_LZMOTOR];

/**
 * @brief 初始化灵足电机
 */
void LZMotor_init(LZ_Motor_ID_t motor_id)
{
    if (motor_id >= LZ_MOTOR_NUM)
        return;

    uint8_t cantype = motor_id / 6; // 获得电机所在can路
    uint8_t canid = motor_id % 6;   // 得到电机ID值；

    memset(&LZ_Motors[cantype][canid], 0, sizeof(LZ_Motor_t));

    // 设置默认ID
    LZ_Motors[cantype][canid].id = canid + 1;
    LZ_Motors[cantype][canid].master_id = DEFAULT_MASTER_ID;

    // 设置默认模式
    LZ_Motors[cantype][canid].mode = LZ_MODE_MIT;

    LZMotor_enable(motor_id);
}

/**
 * @brief 使能灵足电机
 */
void LZMotor_enable(LZ_Motor_ID_t motor_id)
{
    if (motor_id >= LZ_MOTOR_NUM)
        return;

    uint8_t cantype = motor_id / 6; // 获得电机所在can路
    uint8_t canid = motor_id % 6;   // 得到电机ID值；

    lz_enable_motor(cantype, LZ_Motors[cantype][canid].id);
    LZ_Motors[cantype][canid].mode = LZ_MODE_MIT; // 默认使能后进入MIT模式
}

/**
 * @brief 失能灵足电机
 */
void LZMotor_disable(LZ_Motor_ID_t motor_id)
{
    if (motor_id >= LZ_MOTOR_NUM)
        return;

    uint8_t cantype = motor_id / 6; // 获得电机所在can路
    uint8_t canid = motor_id % 6;   // 得到电机ID值；

    lz_disable_motor(cantype, LZ_Motors[cantype][canid].id);
    LZ_Motors[cantype][canid].mode = LZ_MODE_DISABLE;
}

/**
 * @brief 设置灵足电机控制模式
 */
void LZMotor_set_mode(LZ_Motor_ID_t motor_id, LZ_Mode_t mode)
{
    if (motor_id >= LZ_MOTOR_NUM)
        return;

    uint8_t cantype = motor_id / 6; // 获得电机所在can路
    uint8_t canid = motor_id % 6;   // 得到电机ID值；

    uint8_t mode_val;
    switch (mode)
    {
    case LZ_MODE_MIT:
        mode_val = 0;
        break;
    case LZ_MODE_POSITION:
        mode_val = 1;
        break;
    case LZ_MODE_VELOCITY:
        mode_val = 2;
        break;
    default:
        return;
    }

    lz_set_mode(cantype, LZ_Motors[cantype][canid].id, mode_val);
    LZ_Motors[cantype][canid].mode = mode;
}

/**
 * @brief 设置灵足电机控制参数
 */
void LZMotor_set_params(LZ_Motor_ID_t motor_id, float pos, float vel, float tor, float kp, float kd, float current_limit)
{
    if (motor_id >= LZ_MOTOR_NUM)
        return;

    uint8_t cantype = motor_id / 6; // 获得电机所在can路
    uint8_t canid = motor_id % 6;   // 得到电机ID值；

    LZ_Motors[cantype][canid].pos_set = pos;
    LZ_Motors[cantype][canid].vel_set = vel;
    LZ_Motors[cantype][canid].tor_set = tor;
    LZ_Motors[cantype][canid].kp_set = kp;
    LZ_Motors[cantype][canid].kd_set = kd;
    LZ_Motors[cantype][canid].current_limit = current_limit;
}

/**
 * @brief 发送灵足电机控制命令
 */
void LZMotor_send_command(LZ_Motor_ID_t motor_id)
{
    if (motor_id >= LZ_MOTOR_NUM)
        return;

    uint8_t cantype = motor_id / 6; // 获得电机所在can路
    uint8_t canid = motor_id % 6;   // 得到电机ID值；

    LZ_Motor_t *motor = &LZ_Motors[cantype][canid];

    switch (motor->mode)
    {
    case LZ_MODE_MIT:
        lz_send_mit_params(cantype, motor->id, motor->pos_set, motor->vel_set,
                           motor->kp_set, motor->kd_set, motor->tor_set);
        break;

    case LZ_MODE_POSITION:
        lz_set_position(cantype, motor->id, motor->pos_set, motor->vel_set);
        break;

    case LZ_MODE_VELOCITY:
        lz_set_velocity(cantype, motor->id, motor->vel_set, motor->current_limit);
        break;

    default:
        // 其他模式不发送控制命令
        break;
    }
}

/**
 * @brief 获取电机对象指针
 */
LZ_Motor_t *LZMotor_get(LZ_Motor_ID_t motor_id)
{
    if (motor_id >= LZ_MOTOR_NUM)
        return NULL;

    uint8_t cantype = motor_id / 6; // 获得电机所在can路
    uint8_t canid = motor_id % 6;   // 得到电机ID值；

    return &LZ_Motors[cantype][canid];
}

/**
 * @brief 灵足电机CAN数据处理
 */
void LZMotor_decode_candata(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data)
{
    // 根据说明书，MIT协议反馈帧的ID格式为 (电机ID << 8) | 主机ID
    uint8_t motor_id = (id >> 8) & 0xFF;
    uint8_t master_id = id & 0xFF;

    // 确定CAN总线
    uint8_t can_bus = 0;
    if (hfdcan == &hfdcan2)
        can_bus = 1;
    else if (hfdcan == &hfdcan3)
        can_bus = 2;

    // 查找对应的电机
    for (int i = 0; i < MOTORS_PER_CAN; i++)
    {
        if (LZ_Motors[can_bus][i].id == motor_id && LZ_Motors[can_bus][i].master_id == master_id)
        {
            // 解析反馈数据（根据说明书中的通信类型2格式）
            // 这里需要根据实际反馈数据格式进行解析
            // 示例代码，实际应根据说明书调整
            LZ_Motors[can_bus][i].state.angle = uint_to_float_LZ((data[0] << 8) | data[1], P_MIN, P_MAX, 16);
            LZ_Motors[can_bus][i].state.velocity = uint_to_float_LZ((data[2] << 4) | (data[3] >> 4), V_MIN, V_MAX, 12);
            LZ_Motors[can_bus][i].state.torque = uint_to_float_LZ(((data[3] & 0x0F) << 8) | data[4], T_MIN, T_MAX, 12);
            LZ_Motors[can_bus][i].state.temperature = data[5] * 0.1f; // 假设温度数据在data[5]
            break;
        }
    }
}

#endif // USE_LZMotor

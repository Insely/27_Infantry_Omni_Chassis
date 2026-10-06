/**
 * @file  app_api.c
 * @brief 双板步兵应用层的任务分发、板间数据同步及硬件回调适配。
 */

#include "app_api.h"

#include "robot_param.h"
#include "BoardLink.h"
#include "Global_status.h"
#include "Chassis.h"
#include "Gimbal.h"
#include "Shoot.h"
#include "remote_control.h"
#include "Auto_control.h"
#include "referee_system.h"
#include "supercup.h"
#include "IMU_updata.h"
#include "CRC8_CRC16.h"
#include "ui.h"
#include "cmsis_os2.h"

#include <string.h>

static volatile uint8_t gimbal_ready;
static volatile uint8_t chassis_ready;
static volatile uint8_t shoot_ready;
static uint8_t imu_ready;
static uint8_t ui_sequence;

/* ======================================== 应用状态 ========================================= */

/**
 * @brief 判断云台、底盘和发射机构是否均已完成初始化。
 * @return 全部完成返回 1，否则返回 0。
 */
static uint8_t App_Ready(void)
{
    return gimbal_ready && chassis_ready && shoot_ready;
}

/**
 * @brief 初始化应用公共状态和双板通信。
 * @note 该函数在 main.c 中、启动 FreeRTOS 调度器之前调用。
 */
void App_Init(void)
{
    BoardLink_Init();
    Global.Control.mode = LOCK;
    gimbal_ready = 0U;
    chassis_ready = 0U;
    shoot_ready = 0U;
    imu_ready = 0U;
    ui_sequence = 0U;
}

/**
 * @brief 初始化当前板负责的云台执行机构。
 */
void App_GimbalInit(void)
{
#if ROBOT_TYPE == ROBOT_INFANTRY
    Gimbal_Init();
#endif
    gimbal_ready = 1U;
}

/**
 * @brief 初始化当前板负责的底盘执行机构和底盘 IMU。
 */
void App_ChassisInit(void)
{
#if ROBOT_TYPE == ROBOT_INFANTRY
#if BOARD_CHASSIS
    IMU_Init();
    imu_ready = 1U;
#endif
    Chassis_Init();
#endif
    chassis_ready = 1U;
}

/**
 * @brief 初始化当前板负责的发射执行机构。
 */
void App_ShootInit(void)
{
#if ROBOT_TYPE == ROBOT_INFANTRY
    Shoot_Init();
#endif
    shoot_ready = 1U;
}

/* ======================================== 双板接收 ========================================= */

/**
 * @brief 将 BoardLink 接收数据同步到当前板的业务状态。
 * @note 接收结构由中断更新，因此复制时短暂关闭中断以获得一致快照。
 */
static void Link_Rx(void)
{
    BoardLink_t rx;
    uint32_t irq = __get_PRIMASK();

    __disable_irq();
    rx = BoardLink;
    __set_PRIMASK(irq);

#if BOARD_GIMBAL
    Referee_data.Initial_SPEED = rx.initial_speed;
    Referee_data.Barrel_Heat_17mm = rx.barrel_heat;
    Referee_data.Heat_Limit = rx.heat_limit;
    Referee_data.Launching_Frequency = rx.launching_frequency;
    Referee_data.projectile_allowance_17mm = rx.projectile_allowance_17mm;
#else
    Global.Control.mode = rx.control_mode == BL_LOCK
                              ? LOCK
                              : (rx.control_mode == BL_KEY ? KEY : RC);

    switch (rx.chassis_mode)
    {
    case BL_FLOW:
        Global.Chassis.mode = FLOW;
        break;
    case BL_SPIN_P:
        Global.Chassis.mode = SPIN_P;
        break;
    case BL_SPIN_N:
        Global.Chassis.mode = SPIN_N;
        break;
    default:
        Global.Chassis.mode = NO_FOLLOW;
        break;
    }

    Chassis_SetX(rx.vx);
    Chassis_SetY(rx.vy);
    Chassis_SetR(rx.w);
    Global.Cap.mode = rx.cap_mode ? FULL : Not_FULL;
    Global.Chassis.input.reset = rx.reset;
    Global.Auto.mode = rx.auto_mode ? CAR : NONE;
    Global.Shoot.shoot_mode = (enum shoot_mode_e)rx.shoot_mode;

    switch (rx.trigger_mode)
    {
    case BL_HIGH:
        Global.Shoot.trigger_mode = HIGH;
        break;
    case BL_SINGLE:
        Global.Shoot.trigger_mode = SINGLE;
        break;
    case BL_DEBUG:
        Global.Shoot.trigger_mode = DEBUG_TRIGGER;
        break;
    default:
        Global.Shoot.trigger_mode = TRIGGER_CLOSE;
        break;
    }
    ui_sequence = rx.ui_sequence;
#endif
}

/* ======================================== 双板发送 ========================================= */

/**
 * @brief 将当前板业务状态写入 BoardLink 发送结构。
 */
static void Link_Tx(void)
{
#if BOARD_GIMBAL
    BoardLink.control_mode = Global.Control.mode == LOCK
                                 ? BL_LOCK
                                 : (Global.Control.mode == KEY ? BL_KEY : BL_RC);

    switch (Global.Chassis.mode)
    {
    case FLOW:
        BoardLink.chassis_mode = BL_FLOW;
        break;
    case SPIN_P:
        BoardLink.chassis_mode = BL_SPIN_P;
        break;
    case SPIN_N:
        BoardLink.chassis_mode = BL_SPIN_N;
        break;
    default:
        BoardLink.chassis_mode = BL_NO_FOLLOW;
        break;
    }

    BoardLink.vx = Global.Chassis.input.x;
    BoardLink.vy = Global.Chassis.input.y;
    BoardLink.w = Global.Chassis.input.r;
    BoardLink.yaw_speed_cmd = Gimbal.yaw_speed_set;
    BoardLink.pitch = dm_imu_gimbal.pitch;
    BoardLink.yaw_cnt = dm_imu_gimbal.yaw;
    BoardLink.gyro[0] = dm_imu_gimbal.gyro[0];
    BoardLink.gyro[2] = dm_imu_gimbal.gyro[2];

    switch (Global.Shoot.trigger_mode)
    {
    case HIGH:
        BoardLink.trigger_mode = BL_HIGH;
        break;
    case SINGLE:
        BoardLink.trigger_mode = BL_SINGLE;
        break;
    case DEBUG_TRIGGER:
        BoardLink.trigger_mode = BL_DEBUG;
        break;
    default:
        BoardLink.trigger_mode = BL_CLOSE;
        break;
    }

    BoardLink.shoot_mode = Global.Shoot.shoot_mode;
    BoardLink.auto_mode = Global.Auto.mode == CAR;
    BoardLink.cap_mode = Global.Cap.mode == FULL;
    BoardLink.reset = Global.Chassis.input.reset;
    BoardLink.ui_sequence = ui_sequence;
#else
    /* 沿用原车归一化后的电机位置，不重复叠加 YAW_ZERO。 */
    DM_motor_data_s yaw = DMMotor_GetData(YAWMotor);
    BoardLink.yaw_angle_cnt = yaw.feedback_ready && HAL_GetTick() - yaw.feedback_tick <= 20U
                                  ? normalize_angle(yaw.motor_data.para.pos) * RAD_TO_DEG
                                  : NAN;
    BoardLink.yaw_spd = yaw.motor_data.para.vel * RAD_TO_DEG;
    BoardLink.body_gyro_z = IMU_data.gyro[2];
    BoardLink.initial_speed = Referee_data.Initial_SPEED;
    BoardLink.barrel_heat = Referee_data.Barrel_Heat_17mm;
    BoardLink.heat_limit = Referee_data.Heat_Limit;
    BoardLink.launching_frequency = Referee_data.Launching_Frequency;
    BoardLink.projectile_allowance_17mm = Referee_data.projectile_allowance_17mm;
#endif
}

/* ===================================== FreeRTOS 任务入口 =================================== */

/**
 * @brief 遥控任务周期入口：接收双板数据，并在云台板解析遥控输入。
 */
void App_RemoteStep(void)
{
#if ROBOT_TYPE == ROBOT_INFANTRY
    if (!App_Ready())
        return;

    Link_Rx();
#if BOARD_GIMBAL
    Remote_Tasks();
#endif
#endif
}

/**
 * @brief 云台任务周期入口：完成自瞄、Yaw 外环和云台控制解算。
 */
void App_GimbalStep(void)
{
#if ROBOT_TYPE == ROBOT_INFANTRY
    if (!App_Ready())
        return;

#if BOARD_GIMBAL
    if (Global.Auto.mode != NONE &&
        Global.Auto.input.Auto_control_online > 0 &&
        Global.Auto.input.control_mode)
        Auto_Control();

#endif
    Gimbal_Tasks();
#if BOARD_GIMBAL
    if (Global.Auto.input.Auto_control_online > 0)
        --Global.Auto.input.Auto_control_online;
#endif
#endif
}

/**
 * @brief 底盘任务周期入口：完成全向轮底盘解算和超级电容通信。
 */
void App_ChassisStep(void)
{
#if ROBOT_TYPE == ROBOT_INFANTRY
    if (!App_Ready())
        return;

    Chassis_Tasks();
#if BOARD_CHASSIS
    Supercup_SendData();
#endif
#endif
}

/**
 * @brief 发射任务周期入口：分别控制摩擦轮和拨弹电机。
 */
void App_ShootStep(void)
{
#if ROBOT_TYPE == ROBOT_INFANTRY
    if (App_Ready())
        Shoot_Tasks();
#endif
}

/**
 * @brief 电机发送任务周期入口：按板卡职责发送最终控制量。
 */
void App_MotorStep(void)
{
#if ROBOT_TYPE == ROBOT_INFANTRY
    if (!App_Ready())
        return;

#if BOARD_GIMBAL
    DJIMotor_SendCurrent(CAN_20063508_5_8_ID, DJI_CAN_1);
    DMMotor_SendCtrl(PITCHMotor);
#else
    /* LOCK 状态仍发送零电流帧，防止电机保持上一次输出。 */    
    DMMotor_SendCtrl(YAWMotor);
    osDelay(1);
    DJIMotor_SendCurrent(CAN_20063508_1_4_ID, DJI_CAN_1);
    DJIMotor_SendCurrent(CAN_20063508_1_4_ID, DJI_CAN_3);
    
#endif
#endif
}

/**
 * @brief 双板通信周期入口：整理发送数据并执行协议调度。
 */
void App_LinkStep(void)
{
    if (!App_Ready())
        return;

#if ROBOT_TYPE == ROBOT_SENTRY
    BoardLink.control_mode = BL_LOCK;
    BoardLink.trigger_mode = BL_CLOSE;
#else
    Link_Tx();
#endif
    BoardLink_TxStep();
}

/**
 * @brief 裁判系统任务周期入口，并在底盘板刷新 UI。
 */
void App_RefereeStep(void)
{
#if BOARD_CHASSIS
    static uint8_t initialized;
    static uint8_t last_ui_sequence;
    static unsigned refresh_count;

    Referee_unpack_fifo_data(&referee_fifo, &referee_unpack_obj);
    if (!App_Ready() || Referee_data.robot_id == 0U)
        return;

    ui_self_id = Referee_data.robot_id;
    if (!initialized || last_ui_sequence != ui_sequence || ++refresh_count >= 100U)
    {
        ui_init_helm();
        initialized = 1U;
        refresh_count = 0U;
        last_ui_sequence = ui_sequence;
    }

    Supercapui_change(cap.remain_vol);
    Shootui_change();
    Chassisui_change(Chassis.chassis_yaw_angle);
    Autoui_change();
    ui_updata();
#endif
}

/* ========================================= 硬件回调 ======================================== */

/**
 * @brief 请求裁判系统 UI 在下一个周期重新初始化。
 */
void App_RequestUiReset(void)
{
    ++ui_sequence;
}

/**
 * @brief 1 kHz IMU 更新入口，由 TIM13 中断调用。
 */
void App_ImuStep(void)
{
#if ROBOT_TYPE == ROBOT_INFANTRY
#if BOARD_GIMBAL
    IMU_MatchData(&dm_imu_gimbal);
#else
    if (imu_ready)
        IMU_Updata();
#endif
#endif
}

/**
 * @brief 判断当前状态是否允许刷新独立看门狗。
 */
uint8_t App_WatchdogRefreshAllowed(void)
{
#if ROBOT_TYPE == ROBOT_INFANTRY
    return Global.Chassis.input.reset != 1;
#else
    return 1U;
#endif
}

/**
 * @brief 接收视觉串口或 USB 字节流，并按 SP 帧格式完成组帧和校验。
 * @param data 本次收到的数据首地址。
 * @param len 本次收到的数据长度。
 */
void App_OnVisionBytes(const uint8_t *data, uint16_t len)
{
#if BOARD_GIMBAL && ROBOT_TYPE == ROBOT_INFANTRY
    static uint8_t buffer[sizeof(MINIPC_data_t)];
    static unsigned used;

    for (unsigned i = 0U; i < len; ++i)
    {
        if (used == 0U && data[i] != 'S')
            continue;

        buffer[used++] = data[i];
        if (used == 2U && buffer[1] != 'P')
        {
            used = buffer[1] == 'S';
            buffer[0] = 'S';
            continue;
        }

        if (used == sizeof(buffer))
        {
            if (verify_CRC16_check_sum(buffer, sizeof(buffer)))
            {
                decodeMINIPCdata(&fromMINIPC, buffer, sizeof(buffer));
                Global.Auto.input.Auto_control_online = 20;
                MINIPC_to_STM32();
            }
            used = 0U;
        }
    }
#else
    (void)data;
    (void)len;
#endif
}

/**
 * @brief 发送视觉遥测数据，由 TIM14 中断按原频率触发。
 */
void App_SendVisionTelemetry(void)
{
#if BOARD_GIMBAL && ROBOT_TYPE == ROBOT_INFANTRY
    if (App_Ready())
        STM32_to_MINIPC();
#endif
}

/* Yaw 电机由底盘板驱动，位置外环在云台板解算后通过 BoardLink 下发速度指令。 */
#include "Gimbal.h"
#include "Global_status.h"
#include "BoardLink.h"
#include <math.h>
#include <string.h>

#define YAW_CMD_FREEZE_THRESHOLD_MS 50U
#define YAW_CMD_FREEZE_MIN_CMD_RAD_S 0.10f
#define YAW_CMD_FREEZE_MIN_VEL_RAD_S 0.20f
#define YAW_CMD_FREEZE_MAX_CMD_RAD_S (BOARD_LINK_YAW_SPEED_LIMIT_RAD_S - 0.50f)

Gimbal_t Gimbal;

/*
 * Yaw 卡顿诊断：底盘板会持续收到 0x102，但云台控制任务卡住时，帧内的
 * yaw_speed_cmd 会逐位保持不变。仅在非零指令且电机确实在转时判定，并
 * 排除被限幅器固定为 +/-20 rad/s 的正常饱和输出。
 */
static void Gimbal_UpdateYawDebug(float yaw_cmd, DM_motor_data_s yaw_motor)
{
    static uint8_t initialized = 0U;
    static uint8_t freeze_reported = 0U;
    static uint32_t last_cmd_bits = 0U;
    static uint32_t last_cmd_change_tick = 0U;
    static float yaw_cnt_at_cmd_change = 0.0f;
    uint32_t now = HAL_GetTick();
    uint32_t cmd_bits;

    memcpy(&cmd_bits, &yaw_cmd, sizeof(cmd_bits));

    if (initialized == 0U || cmd_bits != last_cmd_bits)
    {
        if (freeze_reported != 0U)
        {
            yaw_debug.command.freeze_active = 0U;
            yaw_debug.command.freeze_last_end_tick_ms = now;
            yaw_debug.command.freeze_last_duration_ms = now - last_cmd_change_tick;
            yaw_debug.command.freeze_last_end_yaw_cnt = BoardLink.yaw_cnt;
        }

        initialized = 1U;
        freeze_reported = 0U;
        last_cmd_bits = cmd_bits;
        last_cmd_change_tick = now;
        yaw_cnt_at_cmd_change = BoardLink.yaw_cnt;
        yaw_debug.command.same_value_duration_ms = 0U;
        return;
    }

    yaw_debug.command.same_value_duration_ms = now - last_cmd_change_tick;
    if (fabsf(yaw_cmd) >= YAW_CMD_FREEZE_MIN_CMD_RAD_S &&
        fabsf(yaw_cmd) <= YAW_CMD_FREEZE_MAX_CMD_RAD_S &&
        fabsf(yaw_motor.motor_data.para.vel) >= YAW_CMD_FREEZE_MIN_VEL_RAD_S &&
        yaw_debug.command.same_value_duration_ms > yaw_debug.command.same_value_max_duration_ms)
        yaw_debug.command.same_value_max_duration_ms = yaw_debug.command.same_value_duration_ms;

    if (freeze_reported == 0U &&
        yaw_debug.command.same_value_duration_ms >= YAW_CMD_FREEZE_THRESHOLD_MS &&
        fabsf(yaw_cmd) >= YAW_CMD_FREEZE_MIN_CMD_RAD_S &&
        fabsf(yaw_cmd) <= YAW_CMD_FREEZE_MAX_CMD_RAD_S &&
        fabsf(yaw_motor.motor_data.para.vel) >= YAW_CMD_FREEZE_MIN_VEL_RAD_S)
    {
        freeze_reported = 1U;
        yaw_debug.command.freeze_active = 1U;
        yaw_debug.command.freeze_event_count++;
        yaw_debug.command.freeze_last_start_tick_ms = last_cmd_change_tick;
        yaw_debug.command.freeze_last_detect_tick_ms = now;
        yaw_debug.command.freeze_last_cmd = yaw_cmd;
        yaw_debug.command.freeze_last_start_yaw_cnt = yaw_cnt_at_cmd_change;
        yaw_debug.command.freeze_last_detect_yaw_cnt = BoardLink.yaw_cnt;
        yaw_debug.command.freeze_last_actual_velocity = yaw_motor.motor_data.para.vel;
        yaw_debug.command.freeze_last_torque = yaw_motor.motor_data.para.tor;
        yaw_debug.command.freeze_last_imu_gyro_z = BoardLink.gyro[2];
        yaw_debug.command.freeze_last_body_gyro_z = BoardLink.body_gyro_z;
        yaw_debug.command.freeze_last_motor_state = yaw_motor.motor_data.para.state;
    }
}

void Gimbal_Init(void) { GIMBALMotor_init(GIMBAL_YAW_MOTOR_TYPE, YAWMotor); Gimbal.State=NORMALLY; }
void Gimbal_Tasks(void)
{
    static uint8_t task_seen = 0U;
    static uint32_t last_task_tick = 0U;
    uint32_t now = HAL_GetTick();
    DM_motor_data_s yaw_motor = GIMBALMotor_get_data(YAWMotor);

    if (task_seen != 0U)
    {
        yaw_debug.task.gimbal_interval_ms = now - last_task_tick;
        if (yaw_debug.task.gimbal_interval_ms > yaw_debug.task.gimbal_max_interval_ms)
            yaw_debug.task.gimbal_max_interval_ms = yaw_debug.task.gimbal_interval_ms;
        if (yaw_debug.task.gimbal_interval_ms > 10U)
        {
            yaw_debug.task.gimbal_gap_over_10ms_count++;
            yaw_debug.task.gimbal_last_gap_tick_ms = now;
            yaw_debug.task.gimbal_last_gap_interval_ms = yaw_debug.task.gimbal_interval_ms;
        }
    }
    else
        task_seen = 1U;
    last_task_tick = now;

    Gimbal_UpdateYawDebug(BoardLink.yaw_speed_cmd, yaw_motor);

    if (Global.Control.mode != LOCK)
        GIMBALMotor_set(YAWMotor,0,BoardLink.yaw_speed_cmd,0,0,2.3f);
    else GIMBALMotor_set(YAWMotor,0,0,0,0,0);
}
void Gimbal_SetPitchAngle(float angle) { (void)angle; }
void Gimbal_SetYawAngle(float angle) { (void)angle; }
